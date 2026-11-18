#include "robot_ui.h"

#include "application.h"
#include "audio_service.h"
#include "display/lvgl_display/lvgl_theme.h"
#include "robot_face.h"
#include "robot_media.h"
#include "robot_skills.h"
#include "settings.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ssid_manager.h>

#include <algorithm>
#include <cmath>
#include <cstring>

#define TAG "RobotUi"


namespace {

constexpr int kW = 320, kH = 240;
const lv_color_t kBg = lv_color_hex(0x000000);
const lv_color_t kAmber = lv_color_hex(0xF5A623);
const lv_color_t kIvory = lv_color_hex(0xFFF6E0);
const lv_color_t kKey = lv_color_hex(0x262626);

// Keyboard layouts. Words instead of symbols, so the text font covers them.
const char* const kDel = "حذف";
const char* const kOk = "تأیید";
const char* const kEsc = "بستن";
const char* const kSpace = " ";

#define C(n) ((lv_buttonmatrix_ctrl_t)(n))

const char* kMapFa[] = {
    "ض", "ص", "ث", "ق", "ف", "غ", "ع", "ه", "خ", "ح", "ج", "\n",
    "ش", "س", "ی", "ب", "ل", "ا", "ت", "ن", "م", "ک", "گ", "\n",
    "ظ", "ط", "ز", "ر", "ذ", "د", "پ", "و", "چ", "ژ", "آ", "\n",
    "123", "EN", kDel, kSpace, kOk, kEsc, ""};
const lv_buttonmatrix_ctrl_t kCtrlFa[] = {
    C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1),
    C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1),
    C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1),
    C(2), C(2), C(2), C(4), C(2), C(2)};

const char* kMapEn[] = {
    "q", "w", "e", "r", "t", "y", "u", "i", "o", "p", "\n",
    "a", "s", "d", "f", "g", "h", "j", "k", "l", "Aa", "\n",
    "z", "x", "c", "v", "b", "n", "m", ".", "@", "_", "\n",
    "123", "فا", kDel, kSpace, kOk, kEsc, ""};
const char* kMapEnCaps[] = {
    "Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P", "\n",
    "A", "S", "D", "F", "G", "H", "J", "K", "L", "Aa", "\n",
    "Z", "X", "C", "V", "B", "N", "M", ".", "@", "_", "\n",
    "123", "فا", kDel, kSpace, kOk, kEsc, ""};
const lv_buttonmatrix_ctrl_t kCtrlEn[] = {
    C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1),
    C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1),
    C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1),
    C(2), C(2), C(2), C(4), C(2), C(2)};

const char* kMapNum[] = {
    "1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "\n",
    "-", "/", ":", ";", "(", ")", "!", "?", "\"", "'", "\n",
    "+", "=", "#", "$", "%", "&", "*", ",", "~", "؟", "\n",
    "فا", "EN", kDel, kSpace, kOk, kEsc, ""};
const lv_buttonmatrix_ctrl_t kCtrlNum[] = {
    C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1),
    C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1),
    C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1), C(1),
    C(2), C(2), C(2), C(4), C(2), C(2)};

struct Cb {
    std::function<void()> fn;
};

void ClickCb(lv_event_t* e) {
    auto* cb = static_cast<Cb*>(lv_event_get_user_data(e));
    auto code = lv_event_get_code(e);
    if (code == LV_EVENT_DELETE) {
        delete cb;
    } else if (code == LV_EVENT_CLICKED && cb->fn) {
        cb->fn();
    }
}

}  // namespace

RobotUi::RobotUi(RobotFaceDisplay* display, RobotMedia* media, RobotSkills* skills)
    : display_(display), media_(media), skills_(skills) {
    Settings s("touch", false);
    if (s.GetInt("ax", 0) != 0) {
        swap_ = s.GetInt("swap", 1) != 0;
        ax_ = s.GetInt("ax");
        bx_ = s.GetInt("bx");
        ay_ = s.GetInt("ay");
        by_ = s.GetInt("by");
        cal_ok_ = true;
    }
    DisplayLockGuard lock(display_);
    indev_ = lv_indev_create();
    lv_indev_set_type(indev_, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev_, ReadCb);
    lv_indev_set_user_data(indev_, this);
    timer_ = lv_timer_create(TimerCb, 100, this);
}

extern "C" const lv_font_t font_fa_18;

const lv_font_t* RobotUi::Font() const { return &font_fa_18; }

namespace {
constexpr int kCalPts[4][2] = {{20, 20}, {300, 20}, {20, 220}, {300, 220}};
}

void RobotUi::Map(int rx, int ry, int& x, int& y) const {
    int u = swap_ ? ry : rx, v = swap_ ? rx : ry;
    x = (ax_ * u + bx_) / 1000;
    y = (ay_ * v + by_) / 1000;
}

void RobotUi::Feed(int rx, int ry, bool pressed) {
    if (cal_step_ >= 0) {
        // Calibration: take the raw point when the finger lifts.
        if (pressed) {
            cal_down_ = true;
            cal_last_[0] = rx;
            cal_last_[1] = ry;
        } else if (cal_down_) {
            cal_down_ = false;
            cal_raw_[cal_step_][0] = cal_last_[0];
            cal_raw_[cal_step_][1] = cal_last_[1];
            ESP_LOGI(TAG, "cal %d raw %d,%d", (int)cal_step_, cal_last_[0], cal_last_[1]);
            DisplayLockGuard lock(display_);
            if (++cal_step_ < 4) {
                ShowCalTarget();
            } else {
                FinishCalibration();
            }
        }
        return;
    }
    int x, y;
    Map(rx, ry, x, y);
    tx_ = std::clamp(x, 0, kW - 1);
    ty_ = std::clamp(y, 0, kH - 1);
    pressed_ = pressed;
}

void RobotUi::StartCalibration() {
    DisplayLockGuard lock(display_);
    ShowCalibrate();
}

void RobotUi::ShowCalibrate() {
    lv_obj_t* r = NewRoot();
    screen_ = kCalibrate;
    lv_obj_t* l = lv_label_create(r);
    lv_label_set_text(l, "روی هر نقطه زرد بزنید");
    lv_obj_center(l);
    cal_dot_ = lv_obj_create(r);
    lv_obj_remove_style_all(cal_dot_);
    lv_obj_set_size(cal_dot_, 16, 16);
    lv_obj_set_style_radius(cal_dot_, 8, 0);
    lv_obj_set_style_bg_color(cal_dot_, kAmber, 0);
    lv_obj_set_style_bg_opa(cal_dot_, LV_OPA_COVER, 0);
    cal_down_ = false;
    cal_step_ = 0;
    ShowCalTarget();
}

void RobotUi::ShowCalTarget() {
    lv_obj_set_pos(cal_dot_, kCalPts[cal_step_][0] - 8, kCalPts[cal_step_][1] - 8);
}

void RobotUi::FinishCalibration() {
    cal_step_ = -1;
    cal_dot_ = nullptr;
    auto& p = cal_raw_;
    // Moving right on screen changes which raw axis more?
    int drx = (p[1][0] - p[0][0]) + (p[3][0] - p[2][0]);
    int dry = (p[1][1] - p[0][1]) + (p[3][1] - p[2][1]);
    bool swap = std::abs(dry) > std::abs(drx);
    auto U = [&](int i) { return swap ? p[i][1] : p[i][0]; };
    auto V = [&](int i) { return swap ? p[i][0] : p[i][1]; };
    int ul = (U(0) + U(2)) / 2, ur = (U(1) + U(3)) / 2;
    int vt = (V(0) + V(1)) / 2, vb = (V(2) + V(3)) / 2;
    if (std::abs(ur - ul) < 40 || std::abs(vb - vt) < 40) {
        ESP_LOGW(TAG, "calibration rejected, try again");
        ShowCalibrate();
        return;
    }
    int ax = 280 * 1000 / (ur - ul), ay = 200 * 1000 / (vb - vt);
    int bx = 20 * 1000 - ax * ul, by = 20 * 1000 - ay * vt;
    // Every tap must land near its dot, or a wrong tap would lock the touch.
    for (int i = 0; i < 4; i++) {
        int x = (ax * U(i) + bx) / 1000, y = (ay * V(i) + by) / 1000;
        if (std::abs(x - kCalPts[i][0]) > 30 || std::abs(y - kCalPts[i][1]) > 30) {
            ESP_LOGW(TAG, "calibration point %d off (%d,%d), try again", i, x, y);
            ShowCalibrate();
            return;
        }
    }
    swap_ = swap;
    ax_ = ax;
    bx_ = bx;
    ay_ = ay;
    by_ = by;
    cal_ok_ = true;
    Settings s("touch", true);
    s.SetInt("swap", swap);
    s.SetInt("ax", ax_);
    s.SetInt("bx", bx_);
    s.SetInt("ay", ay_);
    s.SetInt("by", by_);
    ESP_LOGI(TAG, "calibrated swap=%d ax=%d bx=%d ay=%d by=%d", swap, (int)ax_, (int)bx_, (int)ay_, (int)by_);
    ShowMenu();
}

void RobotUi::ReadCb(lv_indev_t* indev, lv_indev_data_t* data) {
    auto* self = static_cast<RobotUi*>(lv_indev_get_user_data(indev));
    data->point.x = self->tx_;
    data->point.y = self->ty_;
    data->state = self->active_ && self->pressed_ ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

lv_obj_t* RobotUi::NewRoot() {
    // Async: this often runs inside a click on a button of the old screen.
    if (root_) lv_obj_delete_async(root_);
    root_ = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(root_);
    lv_obj_set_size(root_, kW, kH);
    lv_obj_set_style_bg_color(root_, kBg, 0);
    lv_obj_set_style_bg_opa(root_, LV_OPA_COVER, 0);
    lv_obj_set_style_text_font(root_, Font(), 0);
    lv_obj_set_style_text_color(root_, kIvory, 0);
    lv_obj_set_style_base_dir(root_, LV_BASE_DIR_AUTO, 0);
    lv_obj_remove_flag(root_, LV_OBJ_FLAG_SCROLLABLE);
    title_ = bar_ = play_btn_label_ = ta_ = kb_ = nullptr;
    for (auto& e : eq_) e = nullptr;
    active_ = true;
    display_->ui_overlay = true;
    return root_;
}

void RobotUi::Close() {
    if (root_) lv_obj_delete_async(root_);
    root_ = nullptr;
    title_ = bar_ = play_btn_label_ = ta_ = kb_ = nullptr;
    for (auto& e : eq_) e = nullptr;
    screen_ = kNone;
    active_ = false;
    pressed_ = false;
    display_->ui_overlay = false;
}

lv_obj_t* RobotUi::AddButton(lv_obj_t* parent, const char* text, int x, int y, int w, int h,
                             std::function<void()> cb, const lv_font_t* font) {
    lv_obj_t* b = lv_button_create(parent);
    lv_obj_set_pos(b, x, y);
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_bg_color(b, kKey, 0);
    lv_obj_set_style_bg_color(b, kAmber, LV_STATE_PRESSED);
    lv_obj_set_style_radius(b, 10, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_t* l = lv_label_create(b);
    lv_label_set_text(l, text);
    if (font) lv_obj_set_style_text_font(l, font, 0);
    lv_obj_center(l);
    lv_obj_add_event_cb(b, ClickCb, LV_EVENT_ALL, new Cb{std::move(cb)});  // freed on delete
    return b;
}

// ---------------------------------------------------------------------------
// Menu
// ---------------------------------------------------------------------------
void RobotUi::OpenMenu() {
    DisplayLockGuard lock(display_);
    ShowMenu();
}

void RobotUi::ShowMenu() {
    lv_obj_t* r = NewRoot();
    screen_ = kMenu;
    const int w = 300, h = 38, x = 10;
    int y = 8;
    AddButton(r, "پخش‌کننده موسیقی", x, y, w, h, [this]() {
        if (!media_->MusicOn()) media_->PlayAll();
        player_closed_ = false;
        ShowPlayer();
    });
    y += h + 8;
    AddButton(r, "پیام به مومو", x, y, w, h, [this]() {
        ShowKeyboard("پیام به مومو", [](const std::string& t) {
            if (t.empty()) return;
            Application::GetInstance().WakeWordInvoke(t);
        });
    });
    y += h + 8;
    AddButton(r, "پیام به گوشی (بله)", x, y, w, h, [this]() {
        if (int code = skills_->PairCode()) {
            // Not linked yet: show the code the owner sends to the bot.
            lv_obj_t* p = NewRoot();
            screen_ = kMenu;
            lv_obj_t* l = lv_label_create(p);
            lv_obj_set_width(l, kW - 20);
            lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
            lv_label_set_text_fmt(l, "این کد را در بله برای ربات بفرستید:\n\n%d", code);
            lv_obj_align(l, LV_ALIGN_TOP_MID, 0, 40);
            AddButton(p, "بستن", 10, kH - 44, kW - 20, 38, [this]() { Close(); });
            return;
        }
        ShowKeyboard("پیام به گوشی", [this](const std::string& t) {
            if (t.empty()) return;
            struct Job {
                RobotSkills* s;
                std::string m;
            };
            xTaskCreate(
                [](void* a) {
                    auto* j = static_cast<Job*>(a);
                    bool ok = j->s->Notify(j->m);
                    ESP_LOGI(TAG, "Bale message %s", ok ? "sent" : "failed");
                    delete j;
                    vTaskDelete(nullptr);
                },
                "bale_send", 6144, new Job{skills_, t}, 3, nullptr);
        });
    });
    y += h + 8;
    AddButton(r, "تنظیم وای‌فای", x, y, 145, h, [this]() { ShowWifi(); });
    AddButton(r, "تنظیم لمس", x + 155, y, 145, h, [this]() { ShowCalibrate(); });
    y += h + 8;
    AddButton(r, "بستن", x, y, w, h, [this]() { Close(); });
}

// ---------------------------------------------------------------------------
// Music player
// ---------------------------------------------------------------------------
void RobotUi::ShowPlayer() {
    lv_obj_t* r = NewRoot();
    screen_ = kPlayer;

    // Equalizer bars, driven by the playback loudness.
    const int bw = 16, gap = 6, x0 = (kW - 12 * bw - 11 * gap) / 2;
    for (int i = 0; i < 12; i++) {
        lv_obj_t* b = lv_obj_create(r);
        lv_obj_remove_style_all(b);
        lv_obj_set_style_bg_color(b, i % 3 == 1 ? kIvory : kAmber, 0);
        lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(b, 4, 0);
        lv_obj_set_size(b, bw, 8);
        lv_obj_set_pos(b, x0 + i * (bw + gap), 100 - 8);
        eq_[i] = b;
        eq_val_[i] = 0;
    }

    title_ = lv_label_create(r);
    lv_obj_set_width(title_, kW - 20);
    lv_obj_set_pos(title_, 10, 110);
    lv_label_set_long_mode(title_, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_style_text_align(title_, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(title_, "");

    bar_ = lv_bar_create(r);
    lv_obj_set_size(bar_, kW - 40, 6);
    lv_obj_set_pos(bar_, 20, 144);
    lv_bar_set_range(bar_, 0, 1000);
    lv_obj_set_style_bg_color(bar_, kKey, 0);
    lv_obj_set_style_bg_color(bar_, kAmber, LV_PART_INDICATOR);

    const lv_font_t* sym = &lv_font_montserrat_24;
    const int by = 168, bh = 56;
    AddButton(r, LV_SYMBOL_PREV, 12, by, 66, bh, [this]() { media_->Prev(); }, sym);
    lv_obj_t* pb = AddButton(r, LV_SYMBOL_PAUSE, 86, by, 74, bh, [this]() { media_->TogglePause(); }, sym);
    play_btn_label_ = lv_obj_get_child(pb, 0);
    AddButton(r, LV_SYMBOL_NEXT, 168, by, 66, bh, [this]() { media_->Next(); }, sym);
    AddButton(r, LV_SYMBOL_STOP, 242, by, 66, bh, [this]() {
        media_->Stop();
        Close();
    }, sym);
    AddButton(r, LV_SYMBOL_LIST, 6, 6, 38, 34, [this]() { ShowPlaylist(); }, &lv_font_montserrat_14);
    // Small "close" at the top right: hides the player, music keeps playing.
    AddButton(r, LV_SYMBOL_CLOSE, kW - 44, 6, 38, 34, [this]() {
        player_closed_ = true;
        Close();
    }, &lv_font_montserrat_14);
    UpdatePlayer();
}

// Scrollable list of all songs; tapping one plays from there.
void RobotUi::ShowPlaylist() {
    auto names = media_->MusicNames();
    std::string now = media_->NowPlaying();
    lv_obj_t* r = NewRoot();
    screen_ = kPlaylist;
    lv_obj_t* list = lv_obj_create(r);
    lv_obj_remove_style_all(list);
    lv_obj_set_pos(list, 0, 0);
    lv_obj_set_size(list, kW, kH - 50);
    lv_obj_add_flag(list, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    lv_obj_t* cur = nullptr;
    for (size_t i = 0; i < names.size(); i++) {
        lv_obj_t* b = AddButton(list, names[i].c_str(), 10, 6 + (int)i * 44, kW - 20, 38,
                                [this, i]() { media_->PlayFrom(i); player_closed_ = false; ShowPlayer(); });
        lv_obj_t* l = lv_obj_get_child(b, 0);
        lv_obj_set_width(l, kW - 40);
        lv_label_set_long_mode(l, LV_LABEL_LONG_DOT);
        if (names[i] == now) {
            lv_obj_set_style_bg_color(b, kAmber, 0);
            cur = b;
        }
    }
    if (names.empty()) {
        lv_obj_t* l = lv_label_create(list);
        lv_label_set_text(l, "آهنگی روی کارت حافظه نیست");
        lv_obj_center(l);
    }
    if (cur) lv_obj_scroll_to_view(cur, LV_ANIM_OFF);
    AddButton(r, "برگشت", 10, kH - 44, kW - 20, 38, [this]() {
        if (media_->MusicOn()) ShowPlayer(); else Close();
    });
}

void RobotUi::UpdatePlayer() {
    if (title_ == nullptr) return;
    std::string name = media_->NowPlaying();
    if (name.empty()) name = "...";
    if (strcmp(lv_label_get_text(title_), name.c_str()) != 0) lv_label_set_text(title_, name.c_str());
    lv_bar_set_value(bar_, (int)(media_->Progress() * 1000), LV_ANIM_OFF);
    lv_label_set_text(play_btn_label_, media_->Paused() ? LV_SYMBOL_PLAY : LV_SYMBOL_PAUSE);

    int64_t now = esp_timer_get_time() / 1000;
    float lvl = now - g_audio_output_ms < 300 ? std::min(1.0f, g_audio_output_level / 6000.0f) : 0;
    for (int i = 0; i < 12; i++) {
        // Each bar gets its own wobble so the bars do not move together.
        float wob = 0.55f + 0.45f * sinf(now / (120.0f + i * 23) + i * 1.7f);
        float target = lvl * wob;
        eq_val_[i] += (target - eq_val_[i]) * (target > eq_val_[i] ? 0.6f : 0.25f);
        int h = 6 + (int)(eq_val_[i] * 80);
        lv_obj_set_height(eq_[i], h);
        lv_obj_set_y(eq_[i], 100 - h);
    }
}

// ---------------------------------------------------------------------------
// Keyboard
// ---------------------------------------------------------------------------
void RobotUi::ShowKeyboard(const char* title, std::function<void(const std::string&)> done) {
    lv_obj_t* r = NewRoot();
    screen_ = kKeyboard;
    on_done_ = std::move(done);

    lv_obj_t* t = lv_label_create(r);
    lv_label_set_text(t, title);
    lv_obj_set_style_text_color(t, kAmber, 0);
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 2);

    ta_ = lv_textarea_create(r);
    lv_obj_set_pos(ta_, 4, 28);
    lv_obj_set_size(ta_, kW - 8, 40);
    lv_textarea_set_one_line(ta_, true);
    lv_obj_set_style_bg_color(ta_, lv_color_hex(0x141414), 0);
    lv_obj_set_style_text_color(ta_, kIvory, 0);
    lv_obj_set_style_border_color(ta_, kAmber, 0);
    lv_obj_set_style_border_width(ta_, 1, 0);
    lv_obj_set_style_pad_all(ta_, 6, 0);
    lv_obj_add_state(ta_, LV_STATE_FOCUSED);  // shows the cursor

    kb_ = lv_buttonmatrix_create(r);
    lv_obj_set_pos(kb_, 0, 72);
    lv_obj_set_size(kb_, kW, kH - 72);
    lv_obj_set_style_bg_color(kb_, kBg, 0);
    lv_obj_set_style_border_width(kb_, 0, 0);
    lv_obj_set_style_pad_all(kb_, 2, 0);
    lv_obj_set_style_pad_gap(kb_, 3, 0);
    lv_obj_set_style_bg_color(kb_, kKey, LV_PART_ITEMS);
    lv_obj_set_style_bg_color(kb_, kAmber, (lv_style_selector_t)LV_PART_ITEMS | (lv_style_selector_t)LV_STATE_PRESSED);
    lv_obj_set_style_text_color(kb_, kIvory, LV_PART_ITEMS);
    lv_obj_set_style_radius(kb_, 6, LV_PART_ITEMS);
    lv_obj_set_style_shadow_width(kb_, 0, LV_PART_ITEMS);
    lv_obj_set_style_border_width(kb_, 0, LV_PART_ITEMS);
    lv_obj_add_event_cb(
        kb_, [](lv_event_t* e) { static_cast<RobotUi*>(lv_event_get_user_data(e))->OnKey(); },
        LV_EVENT_VALUE_CHANGED, this);
    SetLayout(0);
}

void RobotUi::SetLayout(int layout) {
    kb_layout_ = layout;
    switch (layout) {
        case 0:
            lv_buttonmatrix_set_map(kb_, kMapFa);
            lv_buttonmatrix_set_ctrl_map(kb_, kCtrlFa);
            break;
        case 1:
            lv_buttonmatrix_set_map(kb_, kMapEn);
            lv_buttonmatrix_set_ctrl_map(kb_, kCtrlEn);
            break;
        case 2:
            lv_buttonmatrix_set_map(kb_, kMapEnCaps);
            lv_buttonmatrix_set_ctrl_map(kb_, kCtrlEn);
            break;
        default:
            lv_buttonmatrix_set_map(kb_, kMapNum);
            lv_buttonmatrix_set_ctrl_map(kb_, kCtrlNum);
            break;
    }
}

void RobotUi::OnKey() {
    uint32_t id = lv_buttonmatrix_get_selected_button(kb_);
    const char* k = lv_buttonmatrix_get_button_text(kb_, id);
    if (k == nullptr) return;
    if (strcmp(k, kDel) == 0) {
        lv_textarea_delete_char(ta_);
    } else if (strcmp(k, kOk) == 0) {
        std::string text = lv_textarea_get_text(ta_);
        auto done = std::move(on_done_);
        on_done_ = nullptr;
        Close();
        if (done) done(text);
    } else if (strcmp(k, kEsc) == 0) {
        on_done_ = nullptr;
        Close();
    } else if (strcmp(k, "فا") == 0) {
        SetLayout(0);
    } else if (strcmp(k, "EN") == 0) {
        SetLayout(1);
    } else if (strcmp(k, "Aa") == 0) {
        SetLayout(kb_layout_ == 1 ? 2 : 1);
    } else if (strcmp(k, "123") == 0) {
        SetLayout(3);
    } else {
        lv_textarea_add_text(ta_, k);
        if (kb_layout_ == 2) SetLayout(1);  // one capital letter, like a phone
    }
}

// ---------------------------------------------------------------------------
// Wi-Fi: scan, pick a network, type the password, save and restart.
// ---------------------------------------------------------------------------
void RobotUi::ShowWifi() {
    lv_obj_t* r = NewRoot();
    screen_ = kWifi;
    lv_obj_t* l = lv_label_create(r);
    lv_label_set_text(l, "در حال جستجوی شبکه‌ها...");
    lv_obj_center(l);
    AddButton(r, "بستن", 10, kH - 44, kW - 20, 38, [this]() { Close(); });
    xTaskCreate(ScanTask, "wifi_scan", 4096, this, 3, nullptr);
}

void RobotUi::ScanTask(void* arg) {
    auto* self = static_cast<RobotUi*>(arg);
    std::vector<std::string> names;
    if (esp_wifi_scan_start(nullptr, true) == ESP_OK) {
        uint16_t n = 20;
        wifi_ap_record_t recs[20];
        if (esp_wifi_scan_get_ap_records(&n, recs) == ESP_OK) {
            for (int i = 0; i < n; i++) {
                std::string s = (const char*)recs[i].ssid;
                if (!s.empty() && std::find(names.begin(), names.end(), s) == names.end()) names.push_back(s);
            }
        }
    }
    {
        DisplayLockGuard lock(self->display_);
        if (self->screen_ == kWifi) {
            self->ssids_ = names;
            lv_obj_t* r = self->NewRoot();
            self->screen_ = kWifi;
            lv_obj_t* list = lv_obj_create(r);
            lv_obj_remove_style_all(list);
            lv_obj_set_size(list, kW, kH - 50);
            lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_style_pad_all(list, 6, 0);
            lv_obj_set_style_pad_row(list, 6, 0);
            lv_obj_add_flag(list, LV_OBJ_FLAG_SCROLLABLE);
            if (names.empty()) {
                lv_obj_t* l = lv_label_create(list);
                lv_label_set_text(l, "شبکه‌ای پیدا نشد");
            }
            for (size_t i = 0; i < names.size(); i++) {
                std::string ssid = names[i];
                lv_obj_t* b = self->AddButton(list, ssid.c_str(), 0, 0, kW - 12, 38, [self, ssid]() {
                    self->ShowKeyboard(("رمز " + ssid).c_str(), [ssid](const std::string& pw) {
                        ESP_LOGI(TAG, "Saving Wi-Fi %s", ssid.c_str());
                        SsidManager::GetInstance().AddSsid(ssid, pw);
                        vTaskDelay(pdMS_TO_TICKS(300));
                        esp_restart();
                    });
                    self->SetLayout(1);  // passwords are mostly Latin
                });
                lv_obj_set_style_text_font(lv_obj_get_child(b, 0), self->Font(), 0);
            }
            self->AddButton(r, "بستن", 10, kH - 44, kW - 20, 38, [self]() { self->Close(); });
        }
    }
    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// Opens the player when music starts and closes it when music stops.
// ---------------------------------------------------------------------------
void RobotUi::TimerCb(lv_timer_t* t) {
    auto* self = static_cast<RobotUi*>(lv_timer_get_user_data(t));
    bool on = self->media_ && self->media_->MusicOn();
    if (!on) {
        self->player_closed_ = false;
        if (self->screen_ == kPlayer) self->Close();
        return;
    }
    if (self->screen_ == kNone && !self->player_closed_ && !self->display_->PhotoMode()) {
        self->ShowPlayer();
    } else if (self->screen_ == kPlayer) {
        self->UpdatePlayer();
    }
}
