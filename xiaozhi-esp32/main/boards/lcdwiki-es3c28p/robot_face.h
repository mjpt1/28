// Mimo robot face for XiaoZhi on the 320x240 LCD, drawn into one canvas with
// an anti-aliased polygon rasterizer: ivory eyes, coloured brows, open smile
// with a red tongue, on a pure black background.
// While the robot sleeps the canvas shows a clock, the Persian (Jalali) date
// and the local weather with animated effects. The text drifts slowly so the
// IPS panel does not keep a ghost image.
#pragma once

#include "display/lcd_display.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>

struct FaceParams {
    float brow_dy;    // brows up (-) / down (+), face px
    float brow_tilt;  // deg, + = angry (inner ends down), - = worried
    float eye_open;   // 0..1
    float wink;       // 0..1 closes the right eye
    float mouth;      // smile height scale, negative = frown
    float mouth_sx;   // smile width scale
    float blush;      // 0..1
    float look_x, look_y;
    float tear;       // 0..1
    float shake;      // px jitter
};

// Live data from the car's OBD-II dongle. -1 / -999 = unknown.
struct CarData {
    enum State { kSearching, kConnecting, kOk, kNotFound } state = kSearching;
    int speed = -1, rpm = -1, coolant = -999, dtc = 0;
    float volt = -1;
};

class RobotFaceDisplay : public SpiLcdDisplay {
public:
    // Called with what the user said (speech-to-text), for local voice commands.
    std::function<void(const std::string&)> on_user_text;
    // Set while a touch screen (menu, player, keyboard) covers the face.
    std::atomic<bool> ui_overlay{false};
    using SpiLcdDisplay::SpiLcdDisplay;

    void SetupUI() override;
    // Always use the bundled Persian font (it has the joined letter forms).
    bool SetTextFont(std::shared_ptr<LvglFont> text_font) override;
    bool SetEmotion(const char* emotion) override;

    // Text is hidden in normal use (only clock, Wi-Fi and battery icons stay).
    // During setup states text is shown so the user can follow instructions.
    void SetStatus(const char* status) override;
    using LcdDisplay::ShowNotification;
    void ShowNotification(const char* notification, int duration_ms = 3000) override;
    void SetChatMessage(const char* role, const char* content) override;

    bool SetExpression(const std::string& name, int hold_ms);
    static const char* ExpressionList();
    void ShowLevel(int pct);

    // Sleep screen: clock, Jalali date and weather.
    void SetClockMode(bool on);
    // Open-Meteo WMO weather code, temperature in C, day/night.
    void SetWeather(int code, float temp_c, bool is_day);

    // Car dashboard (car mode replaces face and clock).
    void SetCarMode(bool on);
    void SetCarData(const CarData& d);

    // Photo slideshow: ShowPhoto copies a 320x240 RGB565 image and fades to it.
    void SetPhotoMode(bool on);
    void ShowPhoto(const uint16_t* rgb565);
    bool PhotoMode() const { return photo_mode_; }

private:
    lv_obj_t* canvas_ = nullptr;
    uint16_t* buf_ = nullptr;
    lv_timer_t* timer_ = nullptr;

    FaceParams cur_{};
    FaceParams tgt_{};
    int64_t start_ms_ = 0;
    int64_t hold_until_ms_ = 0;

    float look_x_ = 0, look_y_ = 0, look_tx_ = 0, look_ty_ = 0;
    int64_t next_look_ms_ = 0, blink_start_ms_ = -1000, next_blink_ms_ = 0;
    float talk_ = 0, talk_tgt_ = 0;
    int64_t next_syllable_ms_ = 0;

    int level_pct_ = 0;
    int64_t level_until_ms_ = 0;
    bool level_drawn_ = false;

    bool clock_mode_ = false;
    bool full_redraw_ = true;
    int frame_ = 0;
    float drift_x_ = 0, drift_y_ = 0, drift_tx_ = 0, drift_ty_ = 0;
    int64_t next_drift_ms_ = 0;
    int weather_code_ = -1;
    float weather_temp_ = 0;
    bool weather_day_ = true;
    float px_[48] = {}, py_[48] = {}, pv_[48] = {};
    int64_t flash_until_ms_ = 0, next_flash_ms_ = 0;

    bool text_mode_ = true;
    bool car_mode_ = false;
    bool photo_mode_ = false;
    uint16_t* photo_old_ = nullptr;
    uint16_t* photo_new_ = nullptr;
    int64_t fade_start_ms_ = -1;
    CarData car_;

    static bool IsSetupState();
    static bool IsClock(const char* s);
    void ApplyTextMode();
    static void OnTimer(lv_timer_t* t);
    void Tick();
    void RenderFace(int64_t now);
    void RenderClock(int64_t now);
    void DrawWeather(int64_t now);
    void DrawLevel(int64_t now);
    void RenderCar(int64_t now);
    void RenderPhoto(int64_t now);
};
