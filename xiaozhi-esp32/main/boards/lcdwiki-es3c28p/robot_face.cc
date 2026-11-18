#include "robot_face.h"

#include "application.h"
#include "audio/audio_service.h"
#include "display/lvgl_display/lvgl_theme.h"
#include "fa_glyphs.h"

#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_random.h>
#include <esp_timer.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <ctime>

#define TAG "RobotFace"

namespace {

constexpr float kPi = 3.14159265f;
constexpr int kFrameMs = 40;
constexpr int kIntroEyes = 900;    // eyes pop in
constexpr int kIntroMouth = 1700;  // mouth drawn left to right
constexpr int64_t kNever = INT64_MAX / 4;

int64_t NowMs() { return esp_timer_get_time() / 1000; }
float Clamp01(float v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }
float Lerp(float a, float b, float t) { return a + (b - a) * t; }
float Rand01() { return (esp_random() & 0xFFFF) / 65535.0f; }
float EaseOutBack(float t) {
    t = Clamp01(t);
    const float c1 = 1.7f, c3 = c1 + 1;
    return 1 + c3 * powf(t - 1, 3) + c1 * powf(t - 1, 2);
}

// ---------------------------------------------------------------------------
// Pixel drawing into the RGB565 canvas
// ---------------------------------------------------------------------------
struct Col {
    uint8_t r, g, b;
};
constexpr Col kBg = {0, 0, 0};
constexpr Col kIvory = {255, 245, 220};
constexpr Col kBrow = {255, 170, 40};  // amber brows
constexpr Col kMouth = {70, 8, 14};    // dark red inside of the mouth
constexpr Col kTongue = {235, 35, 45};  // red tongue
constexpr Col kBlush = {255, 90, 130};
constexpr Col kTear = {120, 190, 255};

int W = 320, H = 240;
uint16_t* g_buf = nullptr;

inline uint16_t To565(Col c) { return ((c.r & 0xF8) << 8) | ((c.g & 0xFC) << 3) | (c.b >> 3); }

inline void Blend(int x, int y, Col c, int a) {
    if (a <= 0) return;
    uint16_t& d = g_buf[y * W + x];
    if (a >= 255) {
        d = To565(c);
        return;
    }
    int r = (d >> 11) << 3, g = ((d >> 5) & 0x3F) << 2, b = (d & 0x1F) << 3;
    r += ((c.r - r) * a) >> 8;
    g += ((c.g - g) * a) >> 8;
    b += ((c.b - b) * a) >> 8;
    d = To565({(uint8_t)r, (uint8_t)g, (uint8_t)b});
}

void FillRect(int x0, int y0, int x1, int y1, Col c) {
    x0 = std::max(0, x0); y0 = std::max(0, y0);
    x1 = std::min(W, x1); y1 = std::min(H, y1);
    uint16_t v = To565(c);
    for (int y = y0; y < y1; y++) std::fill(g_buf + y * W + x0, g_buf + y * W + x1, v);
}

struct Pt {
    float x, y;
};

// Anti-aliased polygon fill: 4 sub-scanlines per row, exact horizontal coverage.
float g_cov[320];
float g_xs[300];
void FillPoly(const Pt* p, int n, Col c, float alpha = 1, float clip_x1 = 1e9f) {
    if (n < 3 || alpha <= 0.004f) return;
    float minx = 1e9f, maxx = -1e9f, miny = 1e9f, maxy = -1e9f;
    for (int i = 0; i < n; i++) {
        minx = std::min(minx, p[i].x); maxx = std::max(maxx, p[i].x);
        miny = std::min(miny, p[i].y); maxy = std::max(maxy, p[i].y);
    }
    maxx = std::min(maxx, clip_x1);
    int x0 = std::max(0, (int)floorf(minx)), x1 = std::min(W - 1, (int)ceilf(maxx));
    int y0 = std::max(0, (int)floorf(miny)), y1 = std::min(H - 1, (int)ceilf(maxy));
    if (x1 < x0 || y1 < y0) return;
    float xmax = std::min((float)W, clip_x1);
    for (int row = y0; row <= y1; row++) {
        std::fill(g_cov + x0, g_cov + x1 + 1, 0.f);
        for (int s = 0; s < 4; s++) {
            float sy = row + (s + 0.5f) * 0.25f;
            int m = 0;
            for (int i = 0, j = n - 1; i < n; j = i++) {
                float ya = p[j].y, yb = p[i].y;
                if ((ya <= sy && yb > sy) || (yb <= sy && ya > sy)) {
                    if (m < 300) g_xs[m++] = p[j].x + (sy - ya) * (p[i].x - p[j].x) / (yb - ya);
                }
            }
            std::sort(g_xs, g_xs + m);
            for (int k = 0; k + 1 < m; k += 2) {
                float a = std::max(0.f, g_xs[k]), b = std::min(xmax, g_xs[k + 1]);
                if (b <= a) continue;
                int ia = (int)a, ib = (int)b;
                if (ia == ib) {
                    g_cov[ia] += (b - a) * 0.25f;
                } else {
                    g_cov[ia] += (ia + 1 - a) * 0.25f;
                    for (int x = ia + 1; x < ib; x++) g_cov[x] += 0.25f;
                    if (ib < W) g_cov[ib] += (b - ib) * 0.25f;
                }
            }
        }
        for (int x = x0; x <= x1; x++) {
            if (g_cov[x] > 0.001f) Blend(x, row, c, (int)(std::min(1.f, g_cov[x]) * alpha * 255));
        }
    }
}

void FillEllipse(float cx, float cy, float rx, float ry, Col c, float alpha = 1, float rot = 0) {
    if (rx < 0.3f || ry < 0.3f) return;
    Pt pts[40];
    float cr = cosf(rot), sr = sinf(rot);
    for (int i = 0; i < 40; i++) {
        float a = i * 2 * kPi / 40;
        float x = cosf(a) * rx, y = sinf(a) * ry;
        pts[i] = {cx + x * cr - y * sr, cy + x * sr + y * cr};
    }
    FillPoly(pts, 40, c, alpha);
}

// Thick line as a quad.
void Line(float x0, float y0, float x1, float y1, float w, Col c, float alpha = 1) {
    float dx = x1 - x0, dy = y1 - y0, l = sqrtf(dx * dx + dy * dy);
    if (l < 0.01f) return;
    float nx = -dy / l * w / 2, ny = dx / l * w / 2;
    Pt q[4] = {{x0 + nx, y0 + ny}, {x1 + nx, y1 + ny}, {x1 - nx, y1 - ny}, {x0 - nx, y0 - ny}};
    FillPoly(q, 4, c, alpha);
}

void Glyph(const FaGlyph& g, int x, int y, Col c, float alpha = 1) {
    for (int j = 0; j < g.h; j++) {
        int yy = y + j;
        if (yy < 0 || yy >= H) continue;
        for (int i = 0; i < g.w; i++) {
            int xx = x + i;
            if (xx < 0 || xx >= W) continue;
            int a = g.a[j * g.w + i];
            if (a) Blend(xx, yy, c, (int)(a * alpha));
        }
    }
}

// Number in Persian digits, drawn left to right; returns width.
int NumberWidth(const FaGlyph* set, int v, int digits, int gap) {
    char s[12];
    snprintf(s, sizeof(s), "%0*d", digits, v);
    int w = 0;
    for (char* p = s; *p; p++) w += set[*p - '0'].w + gap;
    return w - gap;
}
void Number(const FaGlyph* set, int v, int digits, int gap, int x, int y, Col c) {
    char s[12];
    snprintf(s, sizeof(s), "%0*d", digits, v);
    for (char* p = s; *p; p++) {
        Glyph(set[*p - '0'], x, y, c);
        x += set[*p - '0'].w + gap;
    }
}

// Gregorian to Jalali (Persian) calendar.
void ToJalali(int gy, int gm, int gd, int& jy, int& jm, int& jd) {
    static const int kDays[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    int gy2 = gm > 2 ? gy + 1 : gy;
    long days = 355666 + 365L * gy + (gy2 + 3) / 4 - (gy2 + 99) / 100 + (gy2 + 399) / 400 + gd +
                kDays[gm - 1];
    jy = -1595 + 33 * (days / 12053);
    days %= 12053;
    jy += 4 * (days / 1461);
    days %= 1461;
    if (days > 365) {
        jy += (days - 1) / 365;
        days = (days - 1) % 365;
    }
    if (days < 186) {
        jm = 1 + days / 31;
        jd = 1 + days % 31;
    } else {
        jm = 7 + (days - 186) / 30;
        jd = 1 + (days - 186) % 30;
    }
}

// ---------------------------------------------------------------------------
// Face shapes (Mimo design, 240x320 portrait coordinates)
// ---------------------------------------------------------------------------
struct Shape {
    Pt p[100];
    int n = 0;
    void Begin(float x, float y) { n = 0; p[n++] = {x, y}; }
    void Curve(float x1, float y1, float x2, float y2, float x3, float y3) {
        Pt s = p[n - 1];
        for (int i = 1; i <= 24 && n < 100; i++) {
            float t = i / 24.0f, u = 1 - t;
            p[n++] = {u * u * u * s.x + 3 * u * u * t * x1 + 3 * u * t * t * x2 + t * t * t * x3,
                      u * u * u * s.y + 3 * u * u * t * y1 + 3 * u * t * t * y2 + t * t * t * y3};
        }
    }
};

Shape g_smile, g_tongue, g_tooth;

void BuildShapes() {
    g_smile.Begin(93, 197);
    g_smile.Curve(109, 205, 131, 205, 147, 193);
    g_smile.Curve(145, 213, 135, 223, 121, 222);
    g_smile.Curve(107, 222, 98, 213, 93, 197);
    g_tongue.Begin(111, 218);
    g_tongue.Curve(117, 207, 133, 206, 139, 213);
    g_tongue.Curve(132, 221, 121, 224, 111, 218);
    g_tooth.Begin(101, 200);
    g_tooth.Curve(107, 202, 113, 203, 119, 203);
    g_tooth.Curve(119, 210, 115, 212, 111, 211);
    g_tooth.Curve(106, 210, 103, 206, 101, 200);
}

// Face space (portrait design) to screen.
constexpr float kScale = 1.45f;
float g_ox = 0, g_oy = 0;  // per-frame offset (bob, shake)
inline Pt Map(float x, float y) {
    return {160 + (x - 118) * kScale + g_ox, 126 + (y - 166) * kScale + g_oy};
}

// Local transform: scale (sx, sy) around (cx, cy), rotate by rot, then move.
void DrawShape(const Shape& s, Col c, float cx, float cy, float sx, float sy, float rot = 0,
               float dx = 0, float dy = 0, float alpha = 1, float clip_x1 = 1e9f) {
    Pt out[100];
    float cr = cosf(rot), sr = sinf(rot);
    for (int i = 0; i < s.n; i++) {
        float x = (s.p[i].x - cx) * sx, y = (s.p[i].y - cy) * sy;
        out[i] = Map(cx + x * cr - y * sr + dx, cy + x * sr + y * cr + dy);
    }
    FillPoly(out, s.n, c, alpha, clip_x1);
}

// Face area that is cleared and sent to the panel each frame.
constexpr int kFX0 = 36, kFY0 = 24, kFX1 = 284, kFY1 = 240;

struct Expr {
    const char* name;
    FaceParams p;
};

//                           brow_dy tilt open wink mouth msx blush lx ly tear shake
constexpr FaceParams kBase = {0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0};

const Expr kExprs[] = {
    {"neutral", {0, 0, 1, 0, 0.8f, 0.95f, 0, 0, 0, 0, 0}},
    {"happy", {-4, 0, 0.9f, 0, 1.15f, 1.05f, 0.4f, 0, 0, 0, 0}},
    {"laughing", {-7, 0, 0.55f, 0, 1.4f, 1.12f, 0.6f, 0, 0, 0, 0}},
    {"funny", {-5, -6, 0.8f, 0, 1.25f, 1.05f, 0.3f, 3, 0, 0, 0}},
    {"relaxed", {0, 0, 0.6f, 0, 0.9f, 1, 0.2f, 0, 0, 0, 0}},
    {"confident", {2, 10, 0.75f, 0, 0.8f, 1, 0, 0, 0, 0, 0}},
    {"cool", {3, 6, 0.5f, 0, 0.6f, 0.9f, 0, 4, 0, 0, 0}},
    {"winking", {-3, 0, 1, 1, 1.1f, 1.05f, 0.3f, 0, 0, 0, 0}},
    {"silly", {-6, -10, 1, 0.5f, 1.3f, 1.1f, 0.2f, -3, 0, 0, 0}},
    {"delicious", {-5, 0, 0.6f, 0, 1.3f, 1.05f, 0.4f, 0, 0, 0, 0}},
    {"hungry", {-9, -8, 1.1f, 0, 0.95f, 0.5f, 0.2f, 5, -6, 0, 0}},
    {"loving", {-4, -6, 0.7f, 0, 1.1f, 1, 1, 0, 0, 0, 0}},
    {"kissy", {-3, 0, 0.6f, 0, 0.6f, 0.45f, 0.8f, 0, 0, 0, 0}},
    {"embarrassed", {-2, -10, 0.8f, 0, 0.4f, 0.7f, 1, -4, 4, 0, 0}},
    {"sad", {-2, -16, 0.75f, 0, -0.6f, 0.85f, 0, 0, 4, 0, 0}},
    {"crying", {-3, -20, 0.6f, 0, -0.8f, 0.85f, 0, 0, 4, 1, 0}},
    {"angry", {6, 22, 0.8f, 0, -0.5f, 0.95f, 0, 0, 0, 0, 1.2f}},
    {"surprised", {-11, 0, 1.15f, 0, 0.9f, 0.5f, 0, 0, -2, 0, 0}},
    {"shocked", {-14, 0, 1.2f, 0, 1.1f, 0.45f, 0, 0, -2, 0, 0.8f}},
    {"thinking", {-5, -10, 0.85f, 0, 0.3f, 0.6f, 0, -5, -5, 0, 0}},
    {"confused", {-3, -14, 0.9f, 0.3f, 0.3f, 0.7f, 0, 3, 0, 0, 0}},
    {"sleepy", {3, 0, 0.12f, 0, 0.25f, 0.6f, 0, 0, 3, 0, 0}},
    {"tired", {3, -6, 0.4f, 0, 0.2f, 0.7f, 0, 0, 3, 0, 0}},
    {"scared", {-9, -18, 1.1f, 0, -0.4f, 0.7f, 0, 0, 0, 0, 1}},
    {"petted", {-6, 0, 0.35f, 0, 1.3f, 1.1f, 1, 0, 0, 0, 0}},
    {"bored", {3, 0, 0.45f, 0, 0.15f, 0.7f, 0, 5, 0, 0, 0}},
};

const FaceParams& FindExpr(const char* name, bool* found = nullptr) {
    if (name != nullptr) {
        for (const auto& e : kExprs) {
            if (strcmp(e.name, name) == 0) {
                if (found) *found = true;
                return e.p;
            }
        }
    }
    if (found) *found = false;
    return kExprs[0].p;
}

// Weather word index in kFaWords (after kFaWeatherBase) and effect kind.
enum Fx { kFxClear, kFxPartly, kFxCloudy, kFxFog, kFxRain, kFxSnow, kFxStorm };
void ClassifyWeather(int code, bool day, int& word, Fx& fx) {
    if (code == 0) { word = day ? 1 : 0; fx = kFxClear; }
    else if (code <= 2) { word = 2; fx = kFxPartly; }
    else if (code == 3) { word = 3; fx = kFxCloudy; }
    else if (code == 45 || code == 48) { word = 4; fx = kFxFog; }
    else if (code >= 51 && code <= 57) { word = 5; fx = kFxRain; }
    else if (code >= 61 && code <= 67) { word = 6; fx = kFxRain; }
    else if ((code >= 71 && code <= 77) || code == 85 || code == 86) { word = 7; fx = kFxSnow; }
    else if (code >= 80 && code <= 82) { word = 8; fx = kFxRain; }
    else if (code >= 95) { word = 9; fx = kFxStorm; }
    else { word = 3; fx = kFxCloudy; }
}

}  // namespace

const char* RobotFaceDisplay::ExpressionList() {
    return "neutral, happy, laughing, funny, relaxed, confident, cool, winking, silly, "
           "delicious, hungry, loving, kissy, embarrassed, sad, crying, angry, surprised, "
           "shocked, thinking, confused, sleepy, tired, scared, bored, petted";
}

extern "C" const lv_font_t font_fa_18;

bool RobotFaceDisplay::SetTextFont(std::shared_ptr<LvglFont>) {
    static auto fa = std::make_shared<LvglBuiltInFont>(&font_fa_18);
    return SpiLcdDisplay::SetTextFont(fa);
}

void RobotFaceDisplay::SetupUI() {
    if (setup_ui_called_) return;
    SpiLcdDisplay::SetupUI();
    SetTextFont(nullptr);

    auto* dark = LvglThemeManager::GetInstance().GetTheme("dark");
    if (dark != nullptr) SetTheme(dark);

    DisplayLockGuard lock(this);
    if (emoji_box_ != nullptr) lv_obj_add_flag(emoji_box_, LV_OBJ_FLAG_HIDDEN);

    W = width_;
    H = height_;
    buf_ = (uint16_t*)heap_caps_malloc(W * H * 2, MALLOC_CAP_SPIRAM);
    if (buf_ == nullptr) {
        ESP_LOGE(TAG, "No memory for face canvas");
        return;
    }
    g_buf = buf_;
    std::fill(buf_, buf_ + W * H, To565(kBg));

    canvas_ = lv_canvas_create(lv_screen_active());
    lv_canvas_set_buffer(canvas_, buf_, W, H, LV_COLOR_FORMAT_RGB565);
    lv_obj_set_pos(canvas_, 0, 0);
    lv_obj_remove_flag(canvas_, LV_OBJ_FLAG_CLICKABLE);
    // Keep status bar, subtitles and popups above the face.
    lv_obj_move_to_index(canvas_, 1);

    BuildShapes();
    for (int i = 0; i < 48; i++) {
        px_[i] = Rand01() * W;
        py_[i] = Rand01() * H;
        pv_[i] = 0.6f + Rand01() * 0.8f;
    }

    cur_ = kExprs[0].p;
    tgt_ = cur_;
    start_ms_ = NowMs();
    next_blink_ms_ = start_ms_ + kIntroMouth + 1500;
    timer_ = lv_timer_create(&RobotFaceDisplay::OnTimer, kFrameMs, this);
}

bool RobotFaceDisplay::SetEmotion(const char* emotion) {
    if (canvas_ == nullptr) return false;
    DisplayLockGuard lock(this);
    if (NowMs() < hold_until_ms_) return true;
    tgt_ = FindExpr(emotion);
    return true;
}

bool RobotFaceDisplay::SetExpression(const std::string& name, int hold_ms) {
    if (canvas_ == nullptr) return false;
    bool found = false;
    const FaceParams& p = FindExpr(name.c_str(), &found);
    if (!found) return false;
    DisplayLockGuard lock(this);
    tgt_ = p;
    hold_until_ms_ = hold_ms > 0 ? NowMs() + hold_ms : 0;
    return true;
}

void RobotFaceDisplay::SetClockMode(bool on) {
    if (canvas_ == nullptr) return;
    DisplayLockGuard lock(this);
    if (clock_mode_ == on) return;
    clock_mode_ = on;
    full_redraw_ = true;
    if (top_bar_ != nullptr) {
        if (on) lv_obj_add_flag(top_bar_, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_remove_flag(top_bar_, LV_OBJ_FLAG_HIDDEN);
    }
}

void RobotFaceDisplay::SetCarMode(bool on) {
    DisplayLockGuard lock(this);
    car_mode_ = on;
    full_redraw_ = true;
    if (top_bar_ != nullptr && on) lv_obj_add_flag(top_bar_, LV_OBJ_FLAG_HIDDEN);
}

void RobotFaceDisplay::SetCarData(const CarData& d) {
    DisplayLockGuard lock(this);
    car_ = d;
}

void RobotFaceDisplay::SetPhotoMode(bool on) {
    if (canvas_ == nullptr) return;
    DisplayLockGuard lock(this);
    if (on && photo_old_ == nullptr) {
        photo_old_ = (uint16_t*)heap_caps_malloc(W * H * 2, MALLOC_CAP_SPIRAM);
        photo_new_ = (uint16_t*)heap_caps_malloc(W * H * 2, MALLOC_CAP_SPIRAM);
        if (photo_old_ == nullptr || photo_new_ == nullptr) return;
    }
    photo_mode_ = on;
    fade_start_ms_ = -1;
    full_redraw_ = true;
    if (top_bar_ != nullptr) {
        if (on || clock_mode_) lv_obj_add_flag(top_bar_, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_remove_flag(top_bar_, LV_OBJ_FLAG_HIDDEN);
    }
    if (on) {
        std::fill(buf_, buf_ + W * H, To565(kBg));
        lv_obj_invalidate(canvas_);
    }
}

void RobotFaceDisplay::ShowPhoto(const uint16_t* img) {
    DisplayLockGuard lock(this);
    if (!photo_mode_ || photo_new_ == nullptr) return;
    memcpy(photo_old_, buf_, W * H * 2);
    memcpy(photo_new_, img, W * H * 2);
    fade_start_ms_ = NowMs();
}

// Cross-fade from the previous picture to the new one over 900 ms.
void RobotFaceDisplay::RenderPhoto(int64_t now) {
    if (fade_start_ms_ < 0) return;
    float t = Clamp01((now - fade_start_ms_) / 900.0f);
    if (t >= 1) {
        memcpy(buf_, photo_new_, W * H * 2);
        fade_start_ms_ = -1;
    } else {
        int a = (int)(t * t * (3 - 2 * t) * 256);  // smoothstep
        for (int i = 0; i < W * H; i++) {
            uint16_t o = photo_old_[i], n = photo_new_[i];
            int r = (o >> 11) + ((((n >> 11) - (o >> 11)) * a) >> 8);
            int g = ((o >> 5) & 0x3F) + (((((n >> 5) & 0x3F) - ((o >> 5) & 0x3F)) * a) >> 8);
            int b = (o & 0x1F) + ((((n & 0x1F) - (o & 0x1F)) * a) >> 8);
            buf_[i] = (r << 11) | (g << 5) | b;
        }
    }
    lv_obj_invalidate(canvas_);
}

void RobotFaceDisplay::SetWeather(int code, float temp_c, bool is_day) {
    DisplayLockGuard lock(this);
    weather_code_ = code;
    weather_temp_ = temp_c;
    weather_day_ = is_day;
}

bool RobotFaceDisplay::IsSetupState() {
    switch (Application::GetInstance().GetDeviceState()) {
        case kDeviceStateWifiConfiguring:
        case kDeviceStateActivating:
        case kDeviceStateUpgrading:
        case kDeviceStateFatalError:
            return true;
        default:
            return false;
    }
}

// "HH:MM" from the status bar clock.
bool RobotFaceDisplay::IsClock(const char* s) {
    if (s == nullptr) return false;
    size_t n = strlen(s);
    if (n < 4 || n > 5) return false;
    for (size_t i = 0; i < n; i++) {
        if (s[i] != ':' && (s[i] < '0' || s[i] > '9')) return false;
    }
    return strchr(s, ':') != nullptr;
}

void RobotFaceDisplay::SetStatus(const char* status) {
    if (IsSetupState() || IsClock(status)) {
        SpiLcdDisplay::SetStatus(status);
        return;
    }
    DisplayLockGuard lock(this);
    if (status_label_ != nullptr) lv_obj_add_flag(status_label_, LV_OBJ_FLAG_HIDDEN);
}

void RobotFaceDisplay::ShowNotification(const char* notification, int duration_ms) {
    if (IsSetupState()) SpiLcdDisplay::ShowNotification(notification, duration_ms);
}

void RobotFaceDisplay::SetChatMessage(const char* role, const char* content) {
    if (IsSetupState()) SpiLcdDisplay::SetChatMessage(role, content);
    if (on_user_text && role && content && strcmp(role, "user") == 0) on_user_text(content);
}

// Runs in the LVGL task. Hides leftover text when setup ends.
void RobotFaceDisplay::ApplyTextMode() {
    bool setup = IsSetupState();
    if (setup == text_mode_) return;
    text_mode_ = setup;
    if (!setup) {
        if (bottom_bar_ != nullptr) lv_obj_add_flag(bottom_bar_, LV_OBJ_FLAG_HIDDEN);
        if (notification_label_ != nullptr) lv_obj_add_flag(notification_label_, LV_OBJ_FLAG_HIDDEN);
        if (status_label_ != nullptr) {
            const char* txt = lv_label_get_text(status_label_);
            if (!IsClock(txt)) lv_obj_add_flag(status_label_, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

void RobotFaceDisplay::OnTimer(lv_timer_t* t) {
    static_cast<RobotFaceDisplay*>(lv_timer_get_user_data(t))->Tick();
}

void RobotFaceDisplay::ShowLevel(int pct) {
    if (canvas_ == nullptr) return;
    DisplayLockGuard lock(this);
    level_pct_ = std::clamp(pct, 0, 100);
    level_until_ms_ = NowMs() + 1500;
}

void RobotFaceDisplay::Tick() {
    ApplyTextMode();
    if (ui_overlay) {
        full_redraw_ = true;  // repaint everything when the overlay closes
        return;
    }
    g_buf = buf_;
    int64_t now = NowMs();
    frame_++;
    if (car_mode_) {
        if (frame_ % 2 == 0) RenderCar(now);
        return;
    }
    if (photo_mode_) {
        RenderPhoto(now);
        return;
    }
    if (clock_mode_) {
        if (full_redraw_ || frame_ % 2 == 0) RenderClock(now);  // 12 fps is enough
        return;
    }
    if (full_redraw_) {
        FillRect(0, 0, W, H, kBg);
        lv_obj_invalidate(canvas_);
        full_redraw_ = false;
    }
    RenderFace(now);
    DrawLevel(now);
}

void RobotFaceDisplay::DrawLevel(int64_t now) {
    bool show = now < level_until_ms_;
    if (!show && !level_drawn_) return;
    const int bw = 12, bh = 140, x = W - 24, y = (H - bh) / 2;
    FillRect(x - 2, y - 2, x + bw + 2, y + bh + 2, kBg);
    if (show) {
        FillRect(x, y, x + bw, y + bh, {28, 36, 48});
        int fh = std::max(4, bh * level_pct_ / 100);
        FillRect(x, y + bh - fh, x + bw, y + bh, kBrow);
    }
    level_drawn_ = show;
    lv_area_t a = {x - 2, y - 2, x + bw + 1, y + bh + 1};
    lv_obj_invalidate_area(canvas_, &a);
}

// ---------------------------------------------------------------------------
// Face
// ---------------------------------------------------------------------------
void RobotFaceDisplay::RenderFace(int64_t now) {
    auto state = Application::GetInstance().GetDeviceState();
    int64_t t = now - start_ms_;

    // Ease every parameter toward its target: smooth morph between moods.
    float* c = reinterpret_cast<float*>(&cur_);
    const float* g = reinterpret_cast<const float*>(&tgt_);
    for (size_t i = 0; i < sizeof(FaceParams) / sizeof(float); i++) c[i] = Lerp(c[i], g[i], 0.15f);

    // Gaze: small random saccades; look at the user while listening.
    if (now >= next_look_ms_) {
        if (state == kDeviceStateListening || Rand01() < 0.35f) {
            look_tx_ = look_ty_ = 0;
        } else {
            look_tx_ = (Rand01() * 2 - 1) * 5;
            look_ty_ = (Rand01() * 2 - 1) * 3;
        }
        next_look_ms_ = now + 1400 + (int)(Rand01() * 2600);
    }
    look_x_ = Lerp(look_x_, look_tx_, 0.12f);
    look_y_ = Lerp(look_y_, look_ty_, 0.12f);

    // Blink.
    if (now >= next_blink_ms_) {
        blink_start_ms_ = now;
        next_blink_ms_ = now + 2200 + (int)(Rand01() * 3400);
    }
    int64_t bt = now - blink_start_ms_;
    float blink = 1;
    if (bt < 100) blink = 1 - bt / 100.0f;
    else if (bt < 145) blink = 0;
    else if (bt < 285) blink = (bt - 145) / 140.0f;

    // Speaking: the mouth follows the loudness of the voice being played.
    // Frames reach the codec ~60 ms before they are heard.
    talk_tgt_ = 0;
    if (now - g_audio_output_ms.load() < 150) {  // speech or music
        float lvl = g_audio_output_level.load();
        talk_tgt_ = Clamp01((lvl - 250) / 3500.0f);
        talk_tgt_ = sqrtf(talk_tgt_);  // small sounds still move the lips
    }
    talk_ = Lerp(talk_, talk_tgt_, talk_tgt_ > talk_ ? 0.7f : 0.35f);

    // Intro: eyes pop in, then the mouth is drawn left to right.
    float pop = t < kIntroEyes ? EaseOutBack((float)t / kIntroEyes) : 1;
    float reveal = Clamp01((float)(t - kIntroEyes) / (kIntroMouth - kIntroEyes));

    float ft = now / 1000.0f;
    g_ox = cur_.shake > 0.05f ? (Rand01() * 2 - 1) * cur_.shake : 0;
    g_oy = sinf(ft * 1.6f) * 2.0f;
    float lx = cur_.look_x + look_x_, ly = cur_.look_y + look_y_;
    bool listening = state == kDeviceStateListening;

    FillRect(kFX0, kFY0, kFX1, kFY1, kBg);


    // Eyes and pupils. Angry (strong inward brow tilt) turns the eyes red.
    float rage = Clamp01((cur_.brow_tilt - 12) / 8);
    Col eye_col = {(uint8_t)Lerp(kIvory.r, 255, rage), (uint8_t)Lerp(kIvory.g, 40, rage),
                   (uint8_t)Lerp(kIvory.b, 30, rage)};
    const float ecy[2] = {155, 156};
    const float ecx[2] = {78, 160};
    for (int i = 0; i < 2; i++) {
        float open = cur_.eye_open * blink * (i == 1 ? 1 - cur_.wink : 1);
        open = std::min(open, 1.25f);
        if (pop < 0.999f) open *= pop;
        float s = pop;
        if (open > 0.08f) {
            // Two small plain ellipses, as in the first design.
            Pt c = Map(ecx[i] + lx * 0.6f, ecy[i] + ly * 0.6f);
            FillEllipse(c.x, c.y, 15 * kScale * s, 23 * kScale * open * s, eye_col);
        } else {
            // Closed: a soft curved line.
            Pt a = Map(ecx[i] - 16, ecy[i]), b = Map(ecx[i] + 16, ecy[i]);
            Line(a.x, a.y, b.x, b.y, 5, eye_col);
        }
    }

    // Blush and tears.
    if (cur_.blush > 0.03f) {
        Pt l = Map(66, 192), r = Map(176, 192);
        FillEllipse(l.x, l.y, 16, 7, kBlush, cur_.blush * 0.55f);
        FillEllipse(r.x, r.y, 16, 7, kBlush, cur_.blush * 0.55f);
    }
    if (cur_.tear > 0.05f) {
        float ph = fmodf(ft * 0.9f, 1.0f);
        Pt d = Map(64, 186 + ph * 30);
        FillEllipse(d.x, d.y, 4, 6, kTear, cur_.tear * (1 - ph));
    }

    // Mouth: open smile with a red tongue; opens wider while speaking.
    float clip = reveal >= 1 ? 1e9f : Map(93 + reveal * 54, 0).x;
    float m = cur_.mouth;
    // Closed at rest; opens and closes only while speaking.
    float sy = talk_ * (m >= 0 ? 1.2f : -0.9f);
    float sx = cur_.mouth_sx * (1 - talk_ * 0.12f);
    float dy = m < 0 ? 24 * -m : 0;  // a frown sits a bit lower
    float mdx = lx * 0.3f;
    if (fabsf(sy) < 0.15f) {
        // Closed mouth: curved red line, bent up for a smile, down for a frown.
        float hw = 27 * sx, bend = std::clamp(m, -1.f, 1.5f) * 6;
        Pt prev = Map(120 - hw + mdx, 199 + dy);
        for (int k = 1; k <= 12; k++) {
            float u = k / 12.0f, v = 2 * u - 1;
            Pt q = Map(120 - hw + 2 * hw * u + mdx, 199 + dy + bend * (1 - v * v));
            if (q.x > clip) break;
            Line(prev.x, prev.y, q.x, q.y, 5, kTongue);
            FillEllipse(q.x, q.y, 2.5f, 2.5f, kTongue);
            prev = q;
        }
    } else {
        DrawShape(g_smile, kMouth, 120, 197, sx, sy, 0, mdx, dy, 1, clip);
    }

    lv_area_t a = {kFX0, kFY0, kFX1 - 1, kFY1 - 1};
    lv_obj_invalidate_area(canvas_, &a);
}

// ---------------------------------------------------------------------------
// Sleep screen: clock, Jalali date and weather
// ---------------------------------------------------------------------------
void RobotFaceDisplay::DrawWeather(int64_t now) {
    if (weather_code_ < 0) return;
    int word;
    Fx fx;
    ClassifyWeather(weather_code_, weather_day_, word, fx);
    float ft = now / 1000.0f;
    const Col cloud = {70, 78, 92}, cloud_dark = {48, 52, 62};

    auto draw_cloud = [&](float x, float y, float s, Col c) {
        FillEllipse(x, y, 26 * s, 16 * s, c);
        FillEllipse(x - 20 * s, y + 5 * s, 16 * s, 11 * s, c);
        FillEllipse(x + 22 * s, y + 4 * s, 18 * s, 12 * s, c);
        FillEllipse(x + 4 * s, y - 10 * s, 15 * s, 13 * s, c);
    };

    bool sun = weather_day_ && (fx == kFxClear || fx == kFxPartly);
    if (sun) {
        float cx = 262, cy = 52;
        for (int r = 4; r >= 1; r--) FillEllipse(cx, cy, 20 + r * 7, 20 + r * 7, {255, 190, 40}, 0.07f);
        float rot = ft * 0.4f;
        for (int k = 0; k < 10; k++) {
            float a = rot + k * 2 * kPi / 10;
            float l0 = 28, l1 = 38 + 4 * sinf(ft * 3 + k);
            Line(cx + cosf(a) * l0, cy + sinf(a) * l0, cx + cosf(a) * l1, cy + sinf(a) * l1, 3,
                 {255, 200, 60}, 0.9f);
        }
        FillEllipse(cx, cy, 22, 22, {255, 205, 60});
        FillEllipse(cx - 6, cy - 7, 8, 6, {255, 235, 150}, 0.6f);
    } else if (!weather_day_ && (fx == kFxClear || fx == kFxPartly)) {
        for (int i = 0; i < 30; i++) {
            float tw = 0.35f + 0.65f * (0.5f + 0.5f * sinf(ft * (1.2f + pv_[i]) + i * 1.7f));
            float sx = fmodf(px_[i] * 1.0f, W), sy = fmodf(py_[i], H * 0.45f);
            FillEllipse(sx, sy, 1.3f, 1.3f, {220, 225, 255}, tw);
        }
        FillEllipse(262, 52, 20, 20, {235, 235, 210});
        FillEllipse(272, 46, 18, 18, kBg);
    }

    if (fx == kFxPartly || fx == kFxCloudy || fx == kFxRain || fx == kFxSnow || fx == kFxStorm) {
        Col cc = (fx == kFxRain || fx == kFxStorm) ? cloud_dark : cloud;
        float drift = fmodf(ft * 6, W + 120);
        draw_cloud(fmodf(40 + drift, W + 120) - 60, 46, 1.0f, cc);
        if (fx != kFxPartly) {
            draw_cloud(fmodf(200 + drift * 0.7f, W + 120) - 60, 36, 0.8f, cc);
            draw_cloud(fmodf(130 + drift * 1.2f, W + 120) - 60, 62, 0.65f, cc);
        } else {
            draw_cloud(fmodf(240 + drift, W + 120) - 60, 66, 0.7f, cc);
        }
    }

    if (fx == kFxRain || fx == kFxStorm) {
        int n = (word == 5) ? 22 : 44;
        for (int i = 0; i < n; i++) {
            py_[i] += 7 * pv_[i] + 3;
            px_[i] -= 1.2f;
            if (py_[i] > H) { py_[i] = 50 + Rand01() * 20; px_[i] = Rand01() * (W + 40); }
            if (px_[i] < -10) px_[i] += W + 20;
            Line(px_[i], py_[i], px_[i] - 2.5f, py_[i] + 11, 1.6f, {110, 160, 255}, 0.75f);
        }
    }
    if (fx == kFxSnow) {
        for (int i = 0; i < 40; i++) {
            py_[i] += 1.2f * pv_[i];
            float x = px_[i] + 8 * sinf(ft * 1.3f + i);
            if (py_[i] > H) { py_[i] = 40 + Rand01() * 30; px_[i] = Rand01() * W; }
            FillEllipse(x, py_[i], 2.2f * pv_[i], 2.2f * pv_[i], {235, 240, 255}, 0.9f);
        }
    }
    if (fx == kFxFog) {
        for (int k = 0; k < 5; k++) {
            float x = fmodf(ft * (8 + k * 3) + k * 90, W + 200) - 100;
            FillEllipse(x, 40 + k * 42, 140, 12, {150, 155, 165}, 0.12f);
        }
    }
    if (fx == kFxStorm) {
        if (now >= next_flash_ms_) {
            flash_until_ms_ = now + 160;
            next_flash_ms_ = now + 2500 + (int)(Rand01() * 4000);
        }
        if (now < flash_until_ms_) {
            Pt bolt[] = {{170, 60}, {150, 110}, {166, 110}, {146, 160}, {184, 98}, {168, 98}, {186, 60}};
            FillRect(0, 0, W, H, {40, 40, 55});  // sky flash
            FillPoly(bolt, 7, {255, 240, 120});
        }
    }
}

void RobotFaceDisplay::RenderClock(int64_t now) {
    full_redraw_ = false;
    FillRect(0, 0, W, H, kBg);

    // Slow drift of the whole layout against burn-in.
    if (now >= next_drift_ms_) {
        drift_tx_ = (Rand01() * 2 - 1) * 26;
        drift_ty_ = (Rand01() * 2 - 1) * 14;
        next_drift_ms_ = now + 40000;
    }
    drift_x_ = Lerp(drift_x_, drift_tx_, 0.01f);
    drift_y_ = Lerp(drift_y_, drift_ty_, 0.01f);
    int ox = (int)lroundf(drift_x_), oy = (int)lroundf(drift_y_);

    DrawWeather(now);

    time_t tt = time(nullptr);
    struct tm tm;
    localtime_r(&tt, &tm);
    bool valid = tm.tm_year + 1900 >= 2024;

    const Col kTime = {225, 228, 240}, kDate = {170, 180, 200}, kAccent = {255, 180, 60};

    // HH:MM in Persian digits.
    int gap = 4;
    int hw = NumberWidth(kFaBig, tm.tm_hour, 2, gap), mw = NumberWidth(kFaBig, tm.tm_min, 2, gap);
    int cw = kFaBig[10].w + 16;
    int total = hw + cw + mw;
    int x = 160 - total / 2 + ox, y = 96 - kFaBig[0].h / 2 + oy;
    if (valid) {
        Number(kFaBig, tm.tm_hour, 2, gap, x, y, kTime);
        bool colon = (now / 500) % 2 == 0;
        Glyph(kFaBig[10], x + hw + 8, y - 6, kTime, colon ? 1 : 0.25f);
        Number(kFaBig, tm.tm_min, 2, gap, x + hw + cw, y, kTime);
    }

    // Date, right to left: weekday, day, month, year.
    if (valid) {
        int jy, jm, jd;
        ToJalali(tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, jy, jm, jd);
        const FaGlyph& wd = kFaWords[kFaWeekdayBase + tm.tm_wday];
        const FaGlyph& mo = kFaWords[jm - 1];
        int dw = NumberWidth(kFaSmall, jd, 1, 1), yw = NumberWidth(kFaSmall, jy, 4, 1);
        const int sp = 9;
        int tw = wd.w + dw + mo.w + yw + 3 * sp;
        int right = 160 + tw / 2 + ox;
        int by = 150 + oy;  // word box top; digits are shorter and sit lower
        int dy = by + (wd.h - kFaSmall[0].h) / 2 + 2;
        Glyph(wd, right - wd.w, by, kAccent);
        right -= wd.w + sp;
        Number(kFaSmall, jd, 1, 1, right - dw, dy, kDate);
        right -= dw + sp;
        Glyph(mo, right - mo.w, by, kDate);
        right -= mo.w + sp;
        Number(kFaSmall, jy, 4, 1, right - yw, dy, kDate);
    }

    // Weather: word on the right, temperature on the left.
    if (weather_code_ >= 0) {
        int word;
        Fx fx;
        ClassifyWeather(weather_code_, weather_day_, word, fx);
        const FaGlyph& wg = kFaWords[kFaWeatherBase + word];
        int temp = (int)lroundf(weather_temp_);
        int tw = NumberWidth(kFaSmall, abs(temp), 1, 1) + kFaSmall[10].w + 2;
        int minus = temp < 0 ? 8 : 0;
        const int sp = 12;
        int total2 = wg.w + sp + tw + minus;
        int right = 160 + total2 / 2 + ox;
        int by = 190 + oy;
        int dy = by + (wg.h - kFaSmall[0].h) / 2 + 2;
        Glyph(wg, right - wg.w, by, kDate);
        right -= wg.w + sp;
        Glyph(kFaSmall[10], right - kFaSmall[10].w, dy - 2, kAccent);
        right -= kFaSmall[10].w + 2;
        int nw = NumberWidth(kFaSmall, abs(temp), 1, 1);
        Number(kFaSmall, abs(temp), 1, 1, right - nw, dy, kAccent);
        if (temp < 0) FillRect(right - nw - 8, dy + 9, right - nw - 2, dy + 11, kAccent);
    }

    lv_obj_invalidate(canvas_);
}

// ---------------------------------------------------------------------------
// Car dashboard: RPM arc around a big speed number, coolant, battery, faults.
// ---------------------------------------------------------------------------
void RobotFaceDisplay::RenderCar(int64_t now) {
    full_redraw_ = false;
    FillRect(0, 0, W, H, kBg);
    const Col kText = {225, 228, 240}, kDim = {120, 130, 150}, kOk = {60, 220, 140},
              kWarn = {255, 170, 40}, kBad = {255, 50, 40};
    // Tiny drift against burn-in.
    int ox = (int)lroundf(sinf(now / 23000.0f) * 5), oy = (int)lroundf(sinf(now / 31000.0f) * 4);
    auto word = [&](int idx, int cx, int y, Col c) {
        const FaGlyph& g = kFaWords[kFaCarBase + idx];
        Glyph(g, cx - g.w / 2 + ox, y + oy, c);
    };

    if (car_.state != CarData::kOk) {
        int idx = car_.state == CarData::kConnecting ? 5 : car_.state == CarData::kNotFound ? 6 : 4;
        word(idx, 160, 100, kText);
        // Spinner.
        for (int k = 0; k < 8; k++) {
            float a = k * kPi / 4 + now / 250.0f;
            FillEllipse(160 + ox + cosf(a) * 18, 160 + oy + sinf(a) * 18, 3, 3, kOk, (k + 1) / 8.0f);
        }
        lv_obj_invalidate(canvas_);
        return;
    }

    // RPM arc: 225 deg sweep, 0..7000 rpm.
    const float cx = 160 + ox, cy = 118 + oy, r = 92;
    const float a0 = kPi * 0.75f, sweep = kPi * 1.5f;
    float frac = car_.rpm < 0 ? 0 : std::min(1.f, car_.rpm / 7000.0f);
    const int seg = 40;
    for (int k = 0; k < seg; k++) {
        float t0 = (float)k / seg, t1 = (k + 0.7f) / seg;
        float b0 = a0 + sweep * t0, b1 = a0 + sweep * t1;
        bool on = t0 < frac;
        Col c = !on ? Col{38, 42, 52} : t0 > 0.78f ? kBad : t0 > 0.6f ? kWarn : kOk;
        Line(cx + cosf(b0) * r, cy + sinf(b0) * r, cx + cosf(b1) * r, cy + sinf(b1) * r, 10, c);
    }

    // Speed.
    int sp = std::max(0, car_.speed);
    int sw = NumberWidth(kFaBig, sp, 1, 4);
    Number(kFaBig, sp, 1, 4, (int)cx - sw / 2, (int)cy - 44, kText);
    {
        const FaGlyph& g = kFaWords[kFaCarBase + 0];
        Glyph(g, (int)cx - g.w / 2, (int)cy + 24, kDim);
    }
    if (car_.rpm >= 0) {
        int rw = NumberWidth(kFaSmall, car_.rpm, 1, 1);
        Number(kFaSmall, car_.rpm, 1, 1, (int)cx - rw / 2, (int)cy + 54, kDim);
    }

    // Bottom row: coolant on the right, battery on the left (RTL).
    bool hot = car_.coolant >= 105;
    bool weak = car_.volt > 0 && car_.volt < 11.8f;
    bool blink = (now / 400) % 2 == 0;
    int by = 205 + oy;
    if (car_.coolant > -100) {
        const FaGlyph& g = kFaWords[kFaCarBase + 1];
        int right = 300 + ox;
        Col c = hot ? (blink ? kBad : kBg) : kText;
        Glyph(g, right - g.w, by, kDim);
        right -= g.w + 8;
        int v = std::abs(car_.coolant);
        int dw = kFaSmall[10].w;
        Glyph(kFaSmall[10], right - dw, by + 2, c);
        right -= dw + 2;
        int nw = NumberWidth(kFaSmall, v, 1, 1);
        Number(kFaSmall, v, 1, 1, right - nw, by + 4, c);
    }
    if (car_.volt > 0) {
        const FaGlyph& g = kFaWords[kFaCarBase + 2];
        int left = 18 + ox;
        int whole = (int)car_.volt, tenth = (int)lroundf((car_.volt - whole) * 10) % 10;
        Col c = weak ? kWarn : kText;
        // Number reads left to right: whole, decimal mark, tenth; word on its right.
        Number(kFaSmall, whole, 1, 1, left, by + 4, c);
        left += NumberWidth(kFaSmall, whole, 1, 1) + 2;
        Glyph(kFaSmall[11], left, by + 4, c);
        left += kFaSmall[11].w + 2;
        Number(kFaSmall, tenth, 1, 1, left, by + 4, c);
        left += kFaSmall[tenth].w + 8;
        Glyph(g, left, by, kDim);
    }

    // Warnings across the top.
    if (hot) {
        word(7, 160, 6, blink ? kBad : kWarn);
    } else if (car_.dtc > 0) {
        const FaGlyph& g = kFaWords[kFaCarBase + 3];
        int nw = NumberWidth(kFaSmall, car_.dtc, 1, 1);
        int x = 160 - (g.w + nw + 8) / 2 + ox;
        Number(kFaSmall, car_.dtc, 1, 1, x, 10 + oy, kWarn);
        Glyph(g, x + nw + 8, 6 + oy, kWarn);
    } else if (weak) {
        word(8, 160, 6, kWarn);
    }

    lv_obj_invalidate(canvas_);
}
