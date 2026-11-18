#include "robot_skills.h"

#include "application.h"
#include "assets/lang_config.h"
#include "board.h"
#include "mcp_server.h"
#include "car_mode.h"
#include "robot_face.h"
#include "settings.h"
#if __has_include("secrets.h")
#include "secrets.h"
#endif

#include <cJSON.h>
#include <esp_log.h>
#include <esp_random.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <algorithm>
#include <cstring>
#include <ctime>

#define TAG "RobotSkills"

namespace {

bool TimeValid() { return time(nullptr) > 1704067200; }  // after 2024-01-01

// Next local hh:mm after now, as UTC seconds.
int64_t NextLocal(int hour, int minute) {
    time_t now = time(nullptr);
    struct tm tm;
    localtime_r(&now, &tm);
    tm.tm_hour = hour;
    tm.tm_min = minute;
    tm.tm_sec = 0;
    time_t t = mktime(&tm);
    if (t <= now) t += 24 * 3600;
    return t;
}

std::string LocalHm(int64_t at) {
    time_t t = at;
    struct tm tm;
    localtime_r(&t, &tm);
    char s[32];
    strftime(s, sizeof(s), "%Y-%m-%d %H:%M", &tm);
    return s;
}

uint8_t Crc8(const uint8_t* d, int n) {
    uint8_t c = 0xFF;
    for (int i = 0; i < n; i++) {
        c ^= d[i];
        for (int b = 0; b < 8; b++) c = (c & 0x80) ? (c << 1) ^ 0x31 : c << 1;
    }
    return c;
}

}  // namespace

RobotSkills::RobotSkills(i2c_master_bus_handle_t i2c, RobotFaceDisplay* display)
    : i2c_(i2c), display_(display) {
#ifdef BALE_BOT_TOKEN
    token_ = BALE_BOT_TOKEN;
#endif
    Settings s("skills", false);
    chat_id_ = atoll(s.GetString("chat_id", "0").c_str());
    pair_code_ = 1000 + esp_random() % 9000;
    Load();
    ProbeSensor();
    AddTools();
}

void RobotSkills::Start() {
    xTaskCreatePinnedToCore(TaskMain, "skills", 8192, this, 2, nullptr, 0);
}

// ---------------------------------------------------------------------------
// Alarms, timers, reminders
// ---------------------------------------------------------------------------
void RobotSkills::Load() {
    Settings s("skills", false);
    cJSON* arr = cJSON_Parse(s.GetString("events", "[]").c_str());
    if (!cJSON_IsArray(arr)) {
        cJSON_Delete(arr);
        return;
    }
    cJSON* it;
    cJSON_ArrayForEach(it, arr) {
        cJSON* id = cJSON_GetObjectItem(it, "id");
        cJSON* at = cJSON_GetObjectItem(it, "at");
        cJSON* label = cJSON_GetObjectItem(it, "label");
        if (!cJSON_IsNumber(id) || !cJSON_IsNumber(at) || !cJSON_IsString(label)) continue;  // broken entry
        Event e;
        e.id = id->valueint;
        e.at = (int64_t)at->valuedouble;
        e.daily = cJSON_IsTrue(cJSON_GetObjectItem(it, "daily"));
        e.label = label->valuestring;
        next_id_ = std::max(next_id_, e.id + 1);
        events_.push_back(e);
    }
    cJSON_Delete(arr);
}

void RobotSkills::Save() {
    cJSON* arr = cJSON_CreateArray();
    for (auto& e : events_) {
        cJSON* o = cJSON_CreateObject();
        cJSON_AddNumberToObject(o, "id", e.id);
        cJSON_AddNumberToObject(o, "at", (double)e.at);
        cJSON_AddBoolToObject(o, "daily", e.daily);
        cJSON_AddStringToObject(o, "label", e.label.c_str());
        cJSON_AddItemToArray(arr, o);
    }
    char* txt = cJSON_PrintUnformatted(arr);
    Settings s("skills", true);
    s.SetString("events", txt);
    cJSON_free(txt);
    cJSON_Delete(arr);
}

int RobotSkills::AddEvent(int64_t at, bool daily, const std::string& label) {
    std::lock_guard<std::mutex> lock(mutex_);
    int id = next_id_++;
    events_.push_back({id, at, daily, label});
    Save();
    return id;
}

std::string RobotSkills::ListEvents() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (events_.empty()) return "No alarms, timers or reminders.";
    std::string out;
    for (auto& e : events_) {
        out += "id " + std::to_string(e.id) + ": " + LocalHm(e.at) + (e.daily ? " (daily)" : "") +
               " - " + e.label + "\n";
    }
    return out;
}

bool RobotSkills::StopRinging() {
    bool was = ringing_;
    ringing_ = false;
    return was;
}

void RobotSkills::Fire(const Event& e) {
    auto& app = Application::GetInstance();
    ESP_LOGI(TAG, "Event %d: %s", e.id, e.label.c_str());
    display_->SetClockMode(false);
    Board::GetInstance().GetBacklight()->RestoreBrightness();
    display_->SetExpression("surprised", 4000);

    // Ring up to 30 s or until the screen is touched.
    ringing_ = true;
    for (int i = 0; i < 20 && ringing_; i++) {
        app.PlaySound(Lang::Sounds::OGG_POPUP);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
    ringing_ = false;

    // The assistant then says it out loud.
    std::string text = "زمان یادآوری رسید: " + e.label + ". این را با لحن دوستانه به من بگو.";
    app.Schedule([text]() { Application::GetInstance().WakeWordInvoke(text); });
}

// ---------------------------------------------------------------------------
// Room climate sensor (optional, I2C 0x44)
// ---------------------------------------------------------------------------
void RobotSkills::ProbeSensor() {
    if (i2c_master_probe(i2c_, 0x44, 50) != ESP_OK) return;
    i2c_device_config_t cfg = {.device_address = 0x44, .scl_speed_hz = 100000};
    if (i2c_master_bus_add_device(i2c_, &cfg, &sht_) != ESP_OK) {
        sht_ = nullptr;
        return;
    }
    float t, rh;
    if (!ReadClimate(t, rh)) {
        sht4x_ = true;  // SHT3x command failed, try SHT4x
        if (!ReadClimate(t, rh)) {
            i2c_master_bus_rm_device(sht_);
            sht_ = nullptr;
            return;
        }
    }
    ESP_LOGI(TAG, "Room sensor %s: %.1f C %.0f%%", sht4x_ ? "SHT4x" : "SHT3x", t, rh);
}

bool RobotSkills::ReadClimate(float& t, float& rh) {
    if (sht_ == nullptr) return false;
    uint8_t cmd3[2] = {0x24, 0x00}, cmd4[1] = {0xFD};
    esp_err_t err = sht4x_ ? i2c_master_transmit(sht_, cmd4, 1, 50)
                           : i2c_master_transmit(sht_, cmd3, 2, 50);
    if (err != ESP_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(20));
    uint8_t d[6];
    if (i2c_master_receive(sht_, d, 6, 50) != ESP_OK) return false;
    if (Crc8(d, 2) != d[2] || Crc8(d + 3, 2) != d[5]) return false;
    uint16_t rt = (d[0] << 8) | d[1], rr = (d[3] << 8) | d[4];
    t = -45 + 175.0f * rt / 65535;
    rh = sht4x_ ? -6 + 125.0f * rr / 65535 : 100.0f * rr / 65535;
    rh = std::clamp(rh, 0.f, 100.f);
    return true;
}

// ---------------------------------------------------------------------------
// Phone messages through a Bale bot (Telegram-compatible API, works in Iran).
// The token comes from secrets.h. The first person who messages the bot
// becomes its owner; only the owner's messages are read out.
// ---------------------------------------------------------------------------
std::string RobotSkills::BaleCall(const std::string& method, const std::string& json) {
    if (token_.empty()) return "";
    auto http = Board::GetInstance().GetNetwork()->CreateHttp(3);
    http->SetTimeout(10000);
    http->SetHeader("Content-Type", "application/json");
    http->SetContent(std::string(json));
    std::string body;
    if (http->Open("POST", "https://tapi.bale.ai/bot" + token_ + "/" + method)) {
        auto code = http->GetStatusCode();
        if (code && *code == 200) body = http->ReadAll();
    }
    http->Close();
    return body;
}

bool RobotSkills::Notify(const std::string& msg) {
    if (chat_id_ == 0) return false;
    cJSON* o = cJSON_CreateObject();
    cJSON_AddNumberToObject(o, "chat_id", (double)chat_id_);
    cJSON_AddStringToObject(o, "text", msg.c_str());
    char* txt = cJSON_PrintUnformatted(o);
    std::string res = BaleCall("sendMessage", txt);
    cJSON_free(txt);
    cJSON_Delete(o);
    return res.find("\"ok\":true") != std::string::npos;
}

void RobotSkills::PollPhone() {
    auto& app = Application::GetInstance();
    if (token_.empty() || app.GetDeviceState() != kDeviceStateIdle) return;
    std::string body = BaleCall("getUpdates", "{\"offset\":" + std::to_string(update_offset_) +
                                                  ",\"timeout\":0,\"limit\":5}");
    cJSON* root = cJSON_Parse(body.c_str());
    if (root == nullptr) return;
    cJSON* arr = cJSON_GetObjectItem(root, "result");
    cJSON* u;
    std::string to_read;
    cJSON_ArrayForEach(u, arr) {
        cJSON* uid = cJSON_GetObjectItem(u, "update_id");
        if (cJSON_IsNumber(uid)) update_offset_ = (int64_t)uid->valuedouble + 1;
        cJSON* msg = cJSON_GetObjectItem(u, "message");
        cJSON* chat = msg ? cJSON_GetObjectItem(msg, "chat") : nullptr;
        cJSON* cid = chat ? cJSON_GetObjectItem(chat, "id") : nullptr;
        cJSON* text = msg ? cJSON_GetObjectItem(msg, "text") : nullptr;
        if (!cJSON_IsNumber(cid) || !cJSON_IsString(text)) continue;
        int64_t id = (int64_t)cid->valuedouble;
        if (chat_id_ == 0) {
            // Pair only with whoever sends the code shown on the robot's screen.
            if (std::to_string(pair_code_) != text->valuestring) continue;
            chat_id_ = id;
            Settings s("skills", true);
            s.SetString("chat_id", std::to_string(id));
            Notify("سلام! من ربات شما هستم. پیام‌هایتان را برایتان می‌خوانم.");
            continue;
        }
        if (id != chat_id_ || text->valuestring[0] == '/') continue;
        std::lock_guard<std::mutex> lock(mutex_);
        inbox_.push_back(text->valuestring);
        to_read = "1";
    }
    cJSON_Delete(root);
    if (!to_read.empty()) {
        // The server accepts only a short wake phrase; the assistant then
        // reads the text through the self.phone.inbox tool.
        std::string t = "پیام جدید";
        display_->SetClockMode(false);
        Board::GetInstance().GetBacklight()->RestoreBrightness();
        app.Schedule([t]() { Application::GetInstance().WakeWordInvoke(t); });
    }
}

// ---------------------------------------------------------------------------
void RobotSkills::TaskMain(void* arg) {
    auto* self = static_cast<RobotSkills*>(arg);
    int tick = 0;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        tick++;
        if (!TimeValid()) continue;
        int64_t now = time(nullptr);

        Event due{};
        bool fire = false;
        {
            std::lock_guard<std::mutex> lock(self->mutex_);
            for (auto it = self->events_.begin(); it != self->events_.end(); ++it) {
                if (it->at > now) continue;
                // Missed by more than 10 minutes (device was off): drop silently.
                bool stale = now - it->at > 600;
                due = *it;
                fire = !stale;
                if (it->daily) {
                    while (it->at <= now) it->at += 24 * 3600;
                } else {
                    self->events_.erase(it);
                }
                self->Save();
                break;
            }
        }
        if (fire) self->Fire(due);

        if (tick % 10 == 0) self->PollPhone();
    }
}

void RobotSkills::AddTools() {
    auto& mcp = McpServer::GetInstance();

    mcp.AddTool("self.alarm.add",
                "Set an alarm or reminder at a local clock time. hour 0-23, minute 0-59. "
                "label: what to say when it rings (e.g. 'wake up', 'take your medicine'). "
                "daily: repeat every day. Use for alarms and reminders at a time of day.",
                PropertyList({Property("hour", kPropertyTypeInteger, 0, 23),
                              Property("minute", kPropertyTypeInteger, 0, 59),
                              Property("label", kPropertyTypeString, std::string("alarm")),
                              Property("daily", kPropertyTypeBoolean, false)}),
                [this](const PropertyList& p) -> ToolResult {
                    if (!TimeValid()) return std::unexpected(std::string("Clock not synced yet"));
                    int64_t at = NextLocal(p["hour"].value<int>(), p["minute"].value<int>());
                    int id = AddEvent(at, p["daily"].value<bool>(), p["label"].value<std::string>());
                    return "Alarm id " + std::to_string(id) + " set for " + LocalHm(at);
                });

    mcp.AddTool("self.timer.add",
                "Start a countdown timer or a reminder after some time. seconds: 1-86400. "
                "label: what to say when it ends (e.g. 'tea is ready').",
                PropertyList({Property("seconds", kPropertyTypeInteger, 60, 1, 86400),
                              Property("label", kPropertyTypeString, std::string("timer"))}),
                [this](const PropertyList& p) -> ToolResult {
                    if (!TimeValid()) return std::unexpected(std::string("Clock not synced yet"));
                    int64_t at = time(nullptr) + p["seconds"].value<int>();
                    int id = AddEvent(at, false, p["label"].value<std::string>());
                    return "Timer id " + std::to_string(id) + " ends at " + LocalHm(at);
                });

    mcp.AddTool("self.alarm.list", "List all alarms, timers and reminders with their ids.",
                PropertyList(), [this](const PropertyList&) -> ToolResult { return ListEvents(); });

    mcp.AddTool("self.alarm.delete", "Delete an alarm, timer or reminder by id (0 = delete all).",
                PropertyList({Property("id", kPropertyTypeInteger, 0, 0, 100000)}),
                [this](const PropertyList& p) -> ToolResult {
                    int id = p["id"].value<int>();
                    std::lock_guard<std::mutex> lock(mutex_);
                    size_t before = events_.size();
                    std::erase_if(events_, [id](const Event& e) { return id == 0 || e.id == id; });
                    Save();
                    return before != events_.size();
                });

    mcp.AddTool("self.room.climate",
                "Read the room temperature (C) and humidity (%) from the robot's sensor.",
                PropertyList(), [this](const PropertyList&) -> ToolResult {
                    float t, rh;
                    if (!ReadClimate(t, rh)) {
                        return std::unexpected(std::string(
                            "No room sensor connected (needs SHT3x/SHT4x on I2C, address 0x44)"));
                    }
                    char s[64];
                    snprintf(s, sizeof(s), "temperature %.1f C, humidity %.0f%%", t, rh);
                    return std::string(s);
                });

    mcp.AddTool("self.phone.notify",
                "Send a message to the user's phone through the Bale messenger. Use when the "
                "user asks to send something to their phone, e.g. a note, list or reminder.",
                PropertyList({Property("message", kPropertyTypeString, std::string(""))}),
                [this](const PropertyList& p) -> ToolResult {
                    if (token_.empty()) return std::unexpected(std::string("Bale bot not configured"));
                    if (chat_id_ == 0) {
                        return std::unexpected(std::string(
                            "Phone not linked yet: open the robot's touch menu, tap the Bale button and send the shown code to the Bale bot"));
                    }
                    if (!Notify(p["message"].value<std::string>())) {
                        return std::unexpected(std::string("Sending failed"));
                    }
                    return true;
                });

    mcp.AddTool("self.phone.inbox",
                "Unread messages the user sent to the robot from their phone (Bale). IMPORTANT: "
                "when the conversation starts with the wake phrase 'پیام جدید' (new message), call "
                "this tool at once and read every message aloud to the user, word for word.",
                PropertyList(), [this](const PropertyList&) -> ToolResult {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (inbox_.empty()) return std::string("No new messages.");
                    std::string out;
                    for (auto& m : inbox_) out += "- " + m + "\n";
                    inbox_.clear();
                    return out;
                });

    mcp.AddTool("self.car.mode_on",
                "Switch the robot to car mode: it connects to the car's OBD-II Wi-Fi dongle and "
                "shows speed, RPM, coolant temperature and battery. The voice assistant is off in "
                "car mode; holding the screen for 3 seconds switches back. Say goodbye first.",
                PropertyList(), [](const PropertyList&) -> ToolResult {
                    Application::GetInstance().Schedule([]() { CarMode::Switch(true); });
                    return true;
                });
}
