// LCDWiki ES3C28P: ESP32-S3, 2.8" ILI9341 IPS 240x320, FT6336G touch, ES8311 codec.
// Same pinout as the Freenove 2.8" board. Screen shows an animated robot face
// (eyes + mouth) instead of emoji images.

#include "adc_battery_monitor.h"
#include "application.h"
#include "button.h"
#include "codecs/es8311_audio_codec.h"
#include "config.h"
#include "display/lcd_display.h"
#include "led/single_led.h"
#include "mcp_server.h"
#include "wifi_board.h"
#include "robot_face.h"
#include "robot_skills.h"
#include "car_mode.h"
#include "robot_media.h"
#include "robot_ui.h"

#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_lcd_ili9341.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cJSON.h>
#include <sys/time.h>

#include <cstdio>
#include <cstring>
#include <ctime>
#include <algorithm>
#include <cstdlib>
#include <string>

#define TAG "LCDWikiES3C28P"

// ILI9341V init for this panel (same as quandong/esp32-s3-spi-2.8).
static const ili9341_lcd_init_cmd_t vendor_specific_init[] = {
    {0xCF, (uint8_t []){0x00, 0xC1, 0x30}, 3, 0},
    {0xED, (uint8_t []){0x64, 0x03, 0x12, 0x81}, 4, 0},
    {0xE8, (uint8_t []){0x85, 0x00, 0x78}, 3, 0},
    {0xCB, (uint8_t []){0x39, 0x2C, 0x00, 0x34, 0x02}, 5, 0},
    {0xF7, (uint8_t []){0x20}, 1, 0},
    {0xEA, (uint8_t []){0x00, 0x00}, 2, 0},
    {0xC0, (uint8_t []){0x13}, 1, 0},
    {0xC1, (uint8_t []){0x13}, 1, 0},
    {0xC5, (uint8_t []){0x22, 0x35}, 2, 0},
    {0xC7, (uint8_t []){0xBD}, 1, 0},
    {0, (uint8_t []){0}, 0x11, 0},
    {0x36, (uint8_t []){0x08}, 1, 0},
    {0xB6, (uint8_t []){0x0A, 0xA2}, 2, 0},
    {0x3A, (uint8_t []){0x55}, 1, 0},
    {0xF6, (uint8_t []){0x01, 0x30}, 2, 0},
    {0xB1, (uint8_t []){0x00, 0x1B}, 2, 0},
    {0xF2, (uint8_t []){0x00}, 1, 0},
    {0x26, (uint8_t []){0x01}, 1, 0},
    {0xE0, (uint8_t []){0x0F, 0x35, 0x31, 0x0B, 0x0E, 0x06, 0x49, 0xA7, 0x33, 0x07, 0x0F, 0x03, 0x0C, 0x0A, 0x00}, 15, 0},
    {0xE1, (uint8_t []){0x00, 0x0A, 0x0F, 0x04, 0x11, 0x08, 0x36, 0x58, 0x4D, 0x07, 0x10, 0x0C, 0x32, 0x34, 0x0F}, 15, 0},
    {0, (uint8_t []){0}, 0x11, 0},
    {0, (uint8_t []){0}, 0x29, 0},
};

// ---------------------------------------------------------------------------
// Board
// ---------------------------------------------------------------------------

// FT6336G: reads first touch point. Coordinates are in raw panel space
// (portrait 240x320); the board maps them to the landscape screen.
class TouchDriver {
public:
    bool Init(i2c_master_bus_handle_t bus, uint8_t addr) {
        i2c_device_config_t cfg = {
            .device_address = addr,
            .scl_speed_hz = 400000,
            .scl_wait_us = 0,
        };
        return i2c_master_bus_add_device(bus, &cfg, &dev_) == ESP_OK;
    }

    bool Read(bool& touched, int& x, int& y) {
        touched = false;
        if (!dev_) return false;
        uint8_t reg = 0x02;
        uint8_t buf[5];
        if (i2c_master_transmit_receive(dev_, &reg, 1, buf, 5, 50) != ESP_OK) return false;
        uint8_t n = buf[0] & 0x0F;
        if (n == 0 || n > 2) return true;
        touched = true;
        x = ((buf[1] & 0x0F) << 8) | buf[2];
        y = ((buf[3] & 0x0F) << 8) | buf[4];
        return true;
    }

private:
    i2c_master_dev_handle_t dev_ = nullptr;
};

class LCDWikiES3C28PBoard : public WifiBoard {
private:
    Button boot_button_;
    RobotFaceDisplay* display_ = nullptr;
    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    TouchDriver touch_;
    AdcBatteryMonitor* battery_ = nullptr;
    RobotSkills* skills_ = nullptr;
    RobotMedia* media_ = nullptr;
    RobotUi* ui_ = nullptr;

    void InitializeI2c() {
        i2c_master_bus_config_t cfg = {
            .i2c_port = AUDIO_CODEC_I2C_NUM,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {.enable_internal_pullup = 1},
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &i2c_bus_));
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = DISPLAY_MIS0_PIN;
        buscfg.sclk_io_num = DISPLAY_SCK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = DISPLAY_SPI_SCLK_HZ;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(LCD_SPI_HOST, &io_config, &panel_io));

        const ili9341_vendor_config_t vendor_config = {
            .init_cmds = &vendor_specific_init[0],
            .init_cmds_size = sizeof(vendor_specific_init) / sizeof(ili9341_lcd_init_cmd_t),
        };
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.vendor_config = (void*)&vendor_config;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        esp_lcd_panel_disp_on_off(panel, true);

        display_ = new RobotFaceDisplay(panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                        DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X,
                                        DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    // Touch gestures:
    //   tap            start / stop chat
    //   double tap     sleep (dim screen, sleepy face) / wake
    //   swipe up/down  volume +/- 10
    //   stroke (rub back and forth)  robot enjoys being petted
    //   hold 6 s       Wi-Fi setup
    bool sleeping_ = false;
    static constexpr int64_t kIdleSleepMs = 60 * 1000;

    void SetSleep(bool on) {
        sleeping_ = on;
        display_->SetClockMode(on);
        if (on) {
            GetBacklight()->SetBrightness(30);
        } else {
            display_->SetExpression("happy", 1500);
            GetBacklight()->RestoreBrightness();
        }
    }

    void ChangeVolume(int delta) {
        auto* codec = GetAudioCodec();
        int v = std::clamp(codec->output_volume() + delta, 0, 100);
        codec->SetOutputVolume(v);
        display_->ShowLevel(v);
    }

    static void TouchTask(void* arg) {
        auto* self = static_cast<LCDWikiES3C28PBoard*>(arg);
        auto& app = Application::GetInstance();
        bool down = false;
        int64_t down_ms = 0, last_tap_ms = 0;
        int sx = 0, sy = 0, lx = 0, ly = 0;  // screen coords
        int path_x = 0, reversals = 0, dir_x = 0;
        bool pending_tap = false;
        int64_t last_activity_ms = esp_timer_get_time() / 1000;

        while (true) {
            bool t = false;
            int rx = 0, ry = 0;
            self->touch_.Read(t, rx, ry);
            int64_t now = esp_timer_get_time() / 1000;
            // Landscape screen point from the saved touch calibration.
            int x = ry, y = rx;
            if (self->ui_) self->ui_->Map(rx, ry, x, y);
            if (t || app.GetDeviceState() != kDeviceStateIdle) last_activity_ms = now;

            // A touch screen (menu, player, keyboard) is open: it gets the touches.
            if (self->ui_ && self->ui_->Active()) {
                self->ui_->Feed(rx, ry, t);
                down = t;
                pending_tap = false;
                last_activity_ms = now;
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }

            if (t && !down && self->media_ && self->media_->PhotosOn()) {
                self->media_->StopPhotos();  // this touch only ends the slideshow
                down = true;
                down_ms = now;
                sx = lx = x;
                sy = ly = y;
                reversals = 3;
            } else if (t && !down && self->skills_ && self->skills_->StopRinging()) {
                down = true;  // this touch only silences the alarm
                down_ms = now;
                sx = lx = x;
                sy = ly = y;
                reversals = 3;
            } else if (t && !down) {
                down = true;
                down_ms = now;
                sx = lx = x;
                sy = ly = y;
                path_x = reversals = dir_x = 0;
            } else if (t && down) {
                int dx = x - lx;
                if (abs(dx) >= 4) {
                    int d = dx > 0 ? 1 : -1;
                    if (dir_x != 0 && d != dir_x) reversals++;
                    dir_x = d;
                    path_x += abs(dx);
                }
                lx = x;
                ly = y;
                // Petting: rubbing back and forth across the face.
                if (path_x > 160 && reversals >= 2) {
                    self->display_->SetExpression("petted", 3000);
                    path_x = 0;
                    reversals = 0;
                }
            } else if (!t && down) {
                down = false;
                int dx = lx - sx, dy = ly - sy;
                int64_t dur = now - down_ms;
                bool stroked = reversals >= 1 || abs(dx) > 40;

                if (dur > 2500 && dur <= 6000 && abs(dx) < 20 && abs(dy) < 20) {
                    // Hold 3-6 s: switch car mode on/off (restarts the board).
                    CarMode::Switch(!CarMode::Enabled());
                } else if (CarMode::Enabled()) {
                    // Car mode: no chat, no other gestures.
                } else if (dur > 6000 && abs(dx) < 20 && abs(dy) < 20) {
                    self->EnterWifiConfigMode();
                } else if (self->ui_ && abs(dx) > 80 && abs(dx) > 2 * abs(dy) && reversals == 0) {
                    self->ui_->OpenMenu();  // swipe left or right: touch menu
                } else if (abs(dy) > 50 && abs(dy) > 2 * abs(dx)) {
                    self->ChangeVolume(dy < 0 ? 10 : -10);
                } else if (!stroked && dur < 400 && abs(dx) < 15 && abs(dy) < 15) {
                    if (pending_tap && now - last_tap_ms < 350) {
                        pending_tap = false;
                        self->SetSleep(!self->sleeping_);
                    } else {
                        pending_tap = true;
                        last_tap_ms = now;
                    }
                }
            }

            // Single tap fires once the double-tap window has passed.
            if (pending_tap && !down && now - last_tap_ms >= 350) {
                pending_tap = false;
                if (CarMode::Enabled()) {
                    // ignore
                } else if (self->sleeping_) {
                    self->SetSleep(false);
                } else {
                    app.Schedule([&app]() { app.ToggleChatState(); });
                }
            }

            // Wake word or chat ends sleep mode.
            if (self->sleeping_ && app.GetDeviceState() != kDeviceStateIdle) {
                self->SetSleep(false);
            }
            // One minute without use: show the clock screen.
            if (self->display_->PhotoMode()) last_activity_ms = now;
            if (!self->sleeping_ && !down && !CarMode::Enabled() &&
                app.GetDeviceState() == kDeviceStateIdle &&
                now - last_activity_ms > kIdleSleepMs) {
                self->SetSleep(true);
            }

            vTaskDelay(pdMS_TO_TICKS(30));
        }
    }

    // ---------------------------------------------------------------------
    // Local time and weather. Location and UTC offset come from the public IP
    // (ip-api.com), weather from Open-Meteo. The HTTP Date header gives true
    // UTC, because the OTA server may set the clock to its own time zone.
    // ---------------------------------------------------------------------
    static std::string HttpGet(const std::string& url, std::string* date = nullptr) {
        auto http = Board::GetInstance().GetNetwork()->CreateHttp(2);
        http->SetTimeout(10000);
        if (!http->Open("GET", url)) return "";
        auto code = http->GetStatusCode();
        if (!code || *code != 200) {
            http->Close();
            return "";
        }
        if (date) *date = http->GetResponseHeader("Date");
        std::string body = http->ReadAll();
        http->Close();
        return body;
    }

    static int64_t DaysFromCivil(int y, int m, int d) {
        y -= m <= 2;
        int64_t era = (y >= 0 ? y : y - 399) / 400;
        int64_t yoe = y - era * 400;
        int64_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
        int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
        return era * 146097 + doe - 719468;
    }

    // "Sat, 07 Nov 2026 10:05:00 GMT" -> sets the system clock to UTC.
    static void SetClockFromHttpDate(const std::string& date) {
        int d, y, H, M, S;
        char mon[4] = {};
        if (sscanf(date.c_str(), "%*3s, %d %3s %d %d:%d:%d", &d, mon, &y, &H, &M, &S) != 6) return;
        static const char* kMon = "JanFebMarAprMayJunJulAugSepOctNovDec";
        const char* p = strstr(kMon, mon);
        if (p == nullptr) return;
        int m = (p - kMon) / 3 + 1;
        time_t utc = DaysFromCivil(y, m, d) * 86400 + H * 3600 + M * 60 + S;
        if (llabs((long long)(utc - time(nullptr))) > 20) {
            struct timeval tv = {.tv_sec = utc, .tv_usec = 0};
            settimeofday(&tv, nullptr);
            ESP_LOGI(TAG, "Clock set from HTTP date: %s", date.c_str());
        }
    }

    static void SetTimeZone(int offset_s) {
        int a = abs(offset_s);
        char tz[32];
        // POSIX TZ: the sign is inverted (UTC+3:30 -> "<+0330>-3:30").
        snprintf(tz, sizeof(tz), "<%c%02d%02d>%c%d:%02d", offset_s >= 0 ? '+' : '-', a / 3600,
                 a % 3600 / 60, offset_s >= 0 ? '-' : '+', a / 3600, a % 3600 / 60);
        setenv("TZ", tz, 1);
        tzset();
    }

    static void WeatherTask(void* arg) {
        auto* self = static_cast<LCDWikiES3C28PBoard*>(arg);
        auto& app = Application::GetInstance();
        float lat = 35.69f, lon = 51.39f;  // fallback: Tehran
        bool located = false;
        SetTimeZone(12600);
        vTaskDelay(pdMS_TO_TICKS(15000));
        while (true) {
            bool ok = false;
            if (app.GetDeviceState() == kDeviceStateIdle) {
                if (!located) {
                    std::string geo = HttpGet("http://ip-api.com/json/?fields=status,lat,lon,offset");
                    cJSON* root = cJSON_Parse(geo.c_str());
                    if (root != nullptr) {
                        cJSON* st = cJSON_GetObjectItem(root, "status");
                        cJSON* la = cJSON_GetObjectItem(root, "lat");
                        cJSON* lo = cJSON_GetObjectItem(root, "lon");
                        cJSON* of = cJSON_GetObjectItem(root, "offset");
                        if (cJSON_IsString(st) && strcmp(st->valuestring, "success") == 0 &&
                            cJSON_IsNumber(la) && cJSON_IsNumber(lo) && cJSON_IsNumber(of)) {
                            lat = la->valuedouble;
                            lon = lo->valuedouble;
                            SetTimeZone(of->valueint);
                            located = true;
                            ESP_LOGI(TAG, "Location %.2f,%.2f offset %d", lat, lon, of->valueint);
                        }
                        cJSON_Delete(root);
                    }
                }
                char url[200];
                snprintf(url, sizeof(url),
                         "http://api.open-meteo.com/v1/forecast?latitude=%.3f&longitude=%.3f"
                         "&current=temperature_2m,weather_code,is_day&timezone=auto",
                         lat, lon);
                std::string date;
                std::string body = HttpGet(url, &date);
                if (!date.empty()) SetClockFromHttpDate(date);
                cJSON* root = cJSON_Parse(body.c_str());
                if (root != nullptr) {
                    cJSON* cur = cJSON_GetObjectItem(root, "current");
                    cJSON* off = cJSON_GetObjectItem(root, "utc_offset_seconds");
                    if (!located && cJSON_IsNumber(off)) SetTimeZone(off->valueint);
                    if (cJSON_IsObject(cur)) {
                        cJSON* tc = cJSON_GetObjectItem(cur, "temperature_2m");
                        cJSON* wc = cJSON_GetObjectItem(cur, "weather_code");
                        cJSON* dy = cJSON_GetObjectItem(cur, "is_day");
                        if (cJSON_IsNumber(tc) && cJSON_IsNumber(wc)) {
                            self->display_->SetWeather(wc->valueint, tc->valuedouble,
                                                       !cJSON_IsNumber(dy) || dy->valueint != 0);
                            ok = true;
                            ESP_LOGI(TAG, "Weather code %d, %.1f C", wc->valueint, tc->valuedouble);
                        }
                    }
                    cJSON_Delete(root);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(ok ? 20 * 60 * 1000 : 60 * 1000));
        }
    }

    void InitializeTouch() {
        gpio_config_t rst = {};
        rst.pin_bit_mask = 1ULL << TOUCH_RST_GPIO;
        rst.mode = GPIO_MODE_OUTPUT;
        gpio_config(&rst);
        gpio_set_level(TOUCH_RST_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(TOUCH_RST_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(100));

        if (!touch_.Init(i2c_bus_, 0x38)) {
            ESP_LOGW(TAG, "Touch init failed");
            return;
        }
        xTaskCreatePinnedToCore(TouchTask, "touch_task", 4096, this, 5, nullptr, 0);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
        // Hold BOOT: redo touch calibration (way out when touch is unusable).
        boot_button_.OnLongPress([this]() {
            if (ui_) ui_->StartCalibration();
        });
    }

    // Lets the assistant set the robot's face, including moods the server
    // emotion list lacks (hungry, tired, scared, bored).
    void InitializeTools() {
        auto& mcp = McpServer::GetInstance();
        mcp.AddTool("self.robot.set_face",
                    std::string("Set the robot's facial expression on the screen. Use it to show "
                                "how you feel, e.g. hungry when talking about food, angry, sad, "
                                "sleepy. expression: one of: ") +
                        RobotFaceDisplay::ExpressionList() +
                        ". hold_seconds: how long to keep it (0 = until next emotion).",
                    PropertyList({Property("expression", kPropertyTypeString, "happy"),
                                  Property("hold_seconds", kPropertyTypeInteger, 8, 0, 120)}),
                    [this](const PropertyList& props) -> ToolResult {
                        auto name = props["expression"].value<std::string>();
                        int hold = props["hold_seconds"].value<int>();
                        if (display_ == nullptr || !display_->SetExpression(name, hold * 1000)) {
                            return std::unexpected(std::string("Unknown expression: ") + name);
                        }
                        return true;
                    });
    }

public:
    LCDWikiES3C28PBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        battery_ = new AdcBatteryMonitor(ADC_UNIT_1, ADC_CHANNEL_8, 200000, 200000, GPIO_NUM_NC);
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeTouch();
        InitializeButtons();
        InitializeTools();
        skills_ = new RobotSkills(i2c_bus_, display_);
        if (!CarMode::Enabled()) skills_->Start();
        if (!CarMode::Enabled()) {
            media_ = new RobotMedia(display_);
            display_->on_user_text = [this](const std::string& t) { media_->OnUserText(t); };
            ui_ = new RobotUi(display_, media_, skills_);
            if (!ui_->Calibrated()) ui_->StartCalibration();  // first boot
        }
        GetBacklight()->RestoreBrightness();
        if (!CarMode::Enabled()) {
            xTaskCreatePinnedToCore(WeatherTask, "weather", 6144, this, 2, nullptr, 0);
        }
    }

    // Car mode replaces the normal Wi-Fi with the OBD dongle's network.
    void StartNetwork() override {
        if (CarMode::Enabled()) CarMode::Run(display_);
        WifiBoard::StartNetwork();
    }

    Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec codec(i2c_bus_, AUDIO_CODEC_I2C_NUM, AUDIO_INPUT_SAMPLE_RATE,
                                      AUDIO_OUTPUT_SAMPLE_RATE, AUDIO_I2S_GPIO_MCLK,
                                      AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT,
                                      AUDIO_I2S_GPIO_DIN, AUDIO_CODEC_PA_PIN,
                                      AUDIO_CODEC_ES8311_ADDR, true, true);
        return &codec;
    }

    Display* GetDisplay() override { return display_; }

    Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        charging = battery_->IsCharging();
        discharging = battery_->IsDischarging();
        level = battery_->GetBatteryLevel();
        return true;
    }
};

DECLARE_BOARD(LCDWikiES3C28PBoard);
