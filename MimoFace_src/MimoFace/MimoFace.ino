#include <Arduino.h>
#include <TFT_eSPI.h>
#include <math.h>

// Display bus, pins and backlight for ES3C28P are in tft_setup.h (this folder),
// taken from the LCDWiki ES3C28P pin table. TFT_eSPI turns the backlight on
// in tft.init() via TFT_BL / TFT_BACKLIGHT_ON, so the sketch leaves it alone.
constexpr int BACKLIGHT_PIN = -1;
constexpr int BACKLIGHT_ON = HIGH;
constexpr int WIDTH = 240, HEIGHT = 320;

TFT_eSPI tft;
TFT_eSprite canvas(&tft);
uint16_t bg, ivory, mint, pupil, sparkle, mouthDark, coral;

struct Point { float x, y; };
struct Shape {
  Point points[256];
  int count = 0;
  void begin(float x, float y) { count = 0; add(x, y); }
  void add(float x, float y) {
    if (count < 256) points[count++] = {x, y};
  }
  void curve(float x1, float y1, float x2, float y2, float x3, float y3) {
    const Point p = points[count - 1];
    for (int i = 1; i <= 24; ++i) {
      float t = i / 24.0f, u = 1 - t;
      add(u*u*u*p.x + 3*u*u*t*x1 + 3*u*t*t*x2 + t*t*t*x3,
          u*u*u*p.y + 3*u*u*t*y1 + 3*u*t*t*y2 + t*t*t*y3);
    }
  }
  // Scanline rasterisation supports concave outlines (brows and smile).
  // Optional vertical scaling is used to close the eyes.
  void fill(uint16_t color, float scaleY = 1, float centerY = 0) const {
    if (count < 3) return;
    float intersections[256];
    for (int row = 0; row < HEIGHT; ++row) {
      float scan = row + 0.5f;
      int n = 0;
      for (int i = 0, j = count - 1; i < count; j = i++) {
        float ya = centerY + (points[j].y - centerY)*scaleY;
        float yb = centerY + (points[i].y - centerY)*scaleY;
        if ((ya <= scan && yb > scan) || (yb <= scan && ya > scan)) {
          intersections[n++] = points[j].x + (scan-ya) *
            (points[i].x-points[j].x)/(yb-ya);
        }
      }
      for (int i = 1; i < n; ++i) {
        float v = intersections[i]; int j = i-1;
        while (j >= 0 && intersections[j] > v) {
          intersections[j+1] = intersections[j]; --j;
        }
        intersections[j+1] = v;
      }
      for (int i = 0; i+1 < n; i += 2) {
        int left = (int)ceilf(intersections[i]-0.5f);
        int right = (int)floorf(intersections[i+1]-0.5f);
        left = max(0, left); right = min(WIDTH-1, right);
        if (right >= left) canvas.drawFastHLine(left, row, right-left+1, color);
      }
    }
  }
};

Shape leftEye, rightEye, leftBrow, rightBrow, smile, tongue, tooth;

void buildShapes() {
  leftEye.begin(48,151);
  leftEye.curve(48,135,58,126,77,126);
  leftEye.curve(97,125,108,136,108,153);
  leftEye.curve(109,173,99,184,80,184);
  leftEye.curve(58,185,48,174,48,151);

  rightEye.begin(132,151);
  rightEye.curve(133,137,143,132,160,132);
  rightEye.curve(179,131,188,141,188,157);
  rightEye.curve(188,174,178,182,161,181);
  rightEye.curve(142,181,132,172,132,151);

  leftBrow.begin(50,126);
  leftBrow.curve(59,114,84,109,98,117);
  leftBrow.curve(83,116,65,120,53,131);
  rightBrow.begin(141,117);
  rightBrow.curve(154,109,176,114,186,126);
  rightBrow.curve(170,120,156,118,144,122);

  smile.begin(93,197);
  smile.curve(109,205,131,205,147,193);
  smile.curve(145,213,135,223,121,222);
  smile.curve(107,222,98,213,93,197);
  tongue.begin(111,218);
  tongue.curve(117,207,133,206,139,213);
  tongue.curve(132,221,121,224,111,218);
  tooth.begin(101,200);
  tooth.curve(107,202,113,203,119,203);
  tooth.curve(119,210,115,212,111,211);
  tooth.curve(106,210,103,206,101,200);
}

void drawPupil(float x, float y, float centerY, float opening, int rx, int ry) {
  int cy = lroundf(centerY + (y-centerY)*opening);
  int radiusY = max(1, (int)lroundf(ry*opening));
  canvas.fillEllipse(lroundf(x), cy, rx, radiusY, pupil);
  if (opening > 0.35f) {
    canvas.fillEllipse(lroundf(x-3), lroundf(cy-7*opening),
                       3, max(1, (int)lroundf(4*opening)), ivory);
    canvas.fillEllipse(lroundf(x+3), lroundf(cy+7*opening),
                       2, max(1, (int)lroundf(2*opening)), sparkle);
  }
}

float gazeX = 0, gazeY = 0, targetX = 0, targetY = 0;
uint32_t nextGaze = 0, nextBlink = 2500, blinkStart = 0;
bool blinking = false;

float updateAnimation(uint32_t now) {
  if ((int32_t)(now-nextGaze) >= 0) {
    // Keep pupils safely inside the eye outlines.
    targetX = random(-5, 4);
    targetY = random(-3, 4);
    nextGaze = now + random(1400, 3400);
  }
  gazeX += (targetX-gazeX)*0.10f;
  gazeY += (targetY-gazeY)*0.10f;
  if (!blinking && (int32_t)(now-nextBlink) >= 0) {
    blinking = true; blinkStart = now;
  }
  if (!blinking) return 1;
  uint32_t elapsed = now-blinkStart;
  if (elapsed < 100) return 1.0f-elapsed/100.0f;
  if (elapsed < 145) return 0;
  if (elapsed < 285) return (elapsed-145)/140.0f;
  blinking = false;
  nextBlink = now + random(2200, 5600);
  return 1;
}

void renderFace(float opening) {
  canvas.fillSprite(bg);
  leftBrow.fill(mint);
  rightBrow.fill(mint);
  if (opening > 0.07f) {
    leftEye.fill(ivory, opening, 155);
    rightEye.fill(ivory, opening, 156);
    drawPupil(87+gazeX, 156+gazeY, 155, opening, 10, 16);
    drawPupil(168+gazeX, 159+gazeY, 156, opening, 9, 14);
  } else {
    canvas.fillRoundRect(49,153,58,4,2,mint);
    canvas.fillRoundRect(134,154,52,4,2,mint);
  }
  smile.fill(mouthDark);
  tongue.fill(coral);
  tooth.fill(ivory);
  canvas.pushSprite(0,0);
}

void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(0); // Portrait 240x320; try 2 if physically upside down.
  if (BACKLIGHT_PIN >= 0) {
    pinMode(BACKLIGHT_PIN, OUTPUT);
    digitalWrite(BACKLIGHT_PIN, BACKLIGHT_ON);
  }
  bg = tft.color565(18,40,49);
  ivory = tft.color565(255,245,220);
  mint = tft.color565(119,218,183);
  pupil = tft.color565(21,62,65);
  sparkle = tft.color565(120,209,181);
  mouthDark = tft.color565(5,21,30);
  coral = tft.color565(250,137,122);
  tft.fillScreen(bg);
  if (tft.width() != WIDTH || tft.height() != HEIGHT) {
    Serial.println("WARNING: configure a 240x320 portrait ILI9341 display.");
  }
  canvas.setColorDepth(16);
  // 153,600 bytes plus overhead; PSRAM can help but is not assumed.
  if (canvas.createSprite(WIDTH,HEIGHT) == nullptr) {
    Serial.println("ERROR: sprite allocation failed. Check available RAM/PSRAM settings.");
    tft.setTextColor(TFT_WHITE, bg);
    tft.drawString("Sprite allocation failed", 8, 12, 2);
    while (true) delay(1000);
  }
  randomSeed(micros());
  buildShapes();
  renderFace(1);
}

void loop() {
  static uint32_t previousFrame = 0;
  uint32_t now = millis();
  if (now-previousFrame < 33) { delay(1); return; }
  previousFrame = now;
  renderFace(updateAnimation(now));
}
