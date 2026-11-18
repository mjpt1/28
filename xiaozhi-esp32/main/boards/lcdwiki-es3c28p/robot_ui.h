// Touch UI drawn with LVGL widgets on top of the face canvas:
//  - a menu (swipe left or right on the face),
//  - a music player that opens while songs play,
//  - an on-screen keyboard (Persian, English, digits) used to message Momo,
//    message the phone through Bale, and type a Wi-Fi password.
// The board's touch task feeds touches here while a screen is open.
#pragma once

#include <lvgl.h>

#include <atomic>
#include <functional>
#include <string>
#include <vector>

class RobotFaceDisplay;
class RobotMedia;
class RobotSkills;

class RobotUi {
public:
    RobotUi(RobotFaceDisplay* display, RobotMedia* media, RobotSkills* skills);

    bool Active() const { return active_; }
    // From the touch task, raw panel coordinates.
    void Feed(int rx, int ry, bool pressed);
    // Raw panel point to screen point, using the saved calibration.
    void Map(int rx, int ry, int& x, int& y) const;
    bool Calibrated() const { return cal_ok_; }
    void StartCalibration();  // tap four corner targets
    void OpenMenu();

private:
    enum Screen { kNone, kMenu, kPlayer, kKeyboard, kWifi, kCalibrate, kPlaylist };

    // Touch calibration: screen = a * raw + b (x1000), swap picks raw axis.
    std::atomic<bool> cal_ok_{false};
    std::atomic<bool> swap_{true};
    std::atomic<int> ax_{1000}, bx_{0}, ay_{1000}, by_{0};
    std::atomic<int> cal_step_{-1};  // corner index while calibrating, -1 otherwise
    int cal_raw_[4][2] = {};
    bool cal_down_ = false;
    int cal_last_[2] = {};
    lv_obj_t* cal_dot_ = nullptr;
    void ShowCalibrate();
    void ShowCalTarget();
    void FinishCalibration();

    RobotFaceDisplay* display_;
    RobotMedia* media_;
    RobotSkills* skills_;

    std::atomic<bool> active_{false};
    std::atomic<int> tx_{0}, ty_{0};
    std::atomic<bool> pressed_{false};

    Screen screen_ = kNone;
    lv_obj_t* root_ = nullptr;  // current screen, child of the top layer
    lv_indev_t* indev_ = nullptr;
    lv_timer_t* timer_ = nullptr;
    bool player_closed_ = false;  // user closed the player for this music run

    // Player widgets.
    lv_obj_t* title_ = nullptr;
    lv_obj_t* bar_ = nullptr;
    lv_obj_t* play_btn_label_ = nullptr;
    lv_obj_t* eq_[12] = {};
    float eq_val_[12] = {};

    // Keyboard.
    lv_obj_t* ta_ = nullptr;
    lv_obj_t* kb_ = nullptr;
    int kb_layout_ = 0;  // 0 Persian, 1 English, 2 English caps, 3 digits
    std::function<void(const std::string&)> on_done_;

    std::vector<std::string> ssids_;

    const lv_font_t* Font() const;
    void Close();
    lv_obj_t* NewRoot();
    lv_obj_t* AddButton(lv_obj_t* parent, const char* text, int x, int y, int w, int h,
                        std::function<void()> cb, const lv_font_t* font = nullptr);

    void ShowMenu();
    void ShowPlayer();
    void ShowPlaylist();
    void ShowKeyboard(const char* title, std::function<void(const std::string&)> done);
    void ShowWifi();
    void SetLayout(int layout);
    void OnKey();
    void UpdatePlayer();

    static void ReadCb(lv_indev_t* indev, lv_indev_data_t* data);
    static void TimerCb(lv_timer_t* t);
    static void ScanTask(void* arg);
};
