#include "car_mode.h"

#include "application.h"
#include "assets/lang_config.h"
#include "robot_face.h"
#include "settings.h"

#include <esp_event.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_netif.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <lwip/inet.h>
#include <lwip/sockets.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <string>

#define TAG "CarMode"

namespace CarMode {

namespace {

EventGroupHandle_t g_events;
constexpr int kGotIp = BIT0, kDisconnected = BIT1;
esp_netif_t* g_netif = nullptr;

void OnEvent(void*, esp_event_base_t base, int32_t id, void*) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(g_events, kGotIp);
        xEventGroupSetBits(g_events, kDisconnected);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupClearBits(g_events, kDisconnected);
        xEventGroupSetBits(g_events, kGotIp);
    }
}

bool LooksLikeObd(const char* ssid) {
    std::string s(ssid);
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    for (const char* k : {"obd", "elm", "vlink", "v-link", "vgate", "konnwei", "car"}) {
        if (s.find(k) != std::string::npos) return true;
    }
    return false;
}

void InitWifi() {
    g_events = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();  // may already exist
    g_netif = esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, OnEvent, nullptr);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, OnEvent, nullptr);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
}

// Scans and joins the strongest OBD-looking network. Open networks first;
// secured ones are tried with the common default password.
bool JoinDongle() {
    wifi_scan_config_t sc = {};
    if (esp_wifi_scan_start(&sc, true) != ESP_OK) return false;
    uint16_t n = 30;
    wifi_ap_record_t recs[30];
    esp_wifi_scan_get_ap_records(&n, recs);
    int best = -1;
    for (int i = 0; i < n; i++) {
        if (LooksLikeObd((const char*)recs[i].ssid) && (best < 0 || recs[i].rssi > recs[best].rssi)) best = i;
    }
    if (best < 0) {
        ESP_LOGW(TAG, "No OBD dongle network found (%d networks)", n);
        return false;
    }
    wifi_config_t wc = {};
    strncpy((char*)wc.sta.ssid, (const char*)recs[best].ssid, sizeof(wc.sta.ssid));
    if (recs[best].authmode != WIFI_AUTH_OPEN) {
        strcpy((char*)wc.sta.password, "12345678");
    }
    ESP_LOGI(TAG, "Joining %s", (const char*)wc.sta.ssid);
    esp_wifi_set_config(WIFI_IF_STA, &wc);
    xEventGroupClearBits(g_events, kGotIp | kDisconnected);
    esp_wifi_connect();
    EventBits_t b = xEventGroupWaitBits(g_events, kGotIp | kDisconnected, pdFALSE, pdFALSE,
                                        pdMS_TO_TICKS(15000));
    if (!(b & kGotIp)) {
        esp_wifi_disconnect();
        return false;
    }
    return true;
}

int Connect(const char* ip, int port) {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) return -1;
    struct timeval tv = {.tv_sec = 3, .tv_usec = 0};
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    sockaddr_in a = {};
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    inet_pton(AF_INET, ip, &a.sin_addr);
    if (connect(s, (sockaddr*)&a, sizeof(a)) != 0) {
        close(s);
        return -1;
    }
    return s;
}

int ConnectElm() {
    // Most v1.5 dongles: 192.168.0.10:35000. Also try the gateway and port 23.
    esp_netif_ip_info_t ip;
    char gw[16] = "192.168.0.10";
    if (esp_netif_get_ip_info(g_netif, &ip) == ESP_OK && ip.gw.addr != 0) {
        esp_ip4addr_ntoa(&ip.gw, gw, sizeof(gw));
    }
    for (const char* host : {"192.168.0.10", (const char*)gw}) {
        for (int port : {35000, 23}) {
            int s = Connect(host, port);
            if (s >= 0) {
                ESP_LOGI(TAG, "ELM327 at %s:%d", host, port);
                return s;
            }
        }
    }
    return -1;
}

// Sends one command and returns the reply up to the '>' prompt, without
// spaces and line breaks. Empty on timeout or socket error.
std::string Cmd(int s, const char* cmd, int wait_ms = 2500) {
    std::string out = std::string(cmd) + "\r";
    if (send(s, out.data(), out.size(), 0) < 0) return "";
    std::string r;
    char buf[128];
    int64_t end = esp_timer_get_time() / 1000 + wait_ms;
    while (esp_timer_get_time() / 1000 < end) {
        int n = recv(s, buf, sizeof(buf), 0);
        if (n <= 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            return "";
        }
        for (int i = 0; i < n; i++) {
            char c = buf[i];
            if (c == '>') return r;
            if (c != ' ' && c != '\r' && c != '\n' && c != '\0') r += (char)toupper(c);
        }
    }
    return r;
}

// Data bytes after "41xx" in a reply; returns count (0 if no data).
int Pid(int s, const char* pid, uint8_t* out, int max) {
    std::string r = Cmd(s, pid);
    std::string key = std::string("41") + (pid + 2);
    size_t p = r.find(key);
    if (p == std::string::npos) return 0;
    p += key.size();
    int n = 0;
    while (n < max && p + 1 < r.size() && isxdigit(r[p]) && isxdigit(r[p + 1])) {
        out[n++] = (uint8_t)strtol(r.substr(p, 2).c_str(), nullptr, 16);
        p += 2;
    }
    return n;
}

}  // namespace

bool Enabled() {
    Settings s("car", false);
    return s.GetBool("on", false);
}

void Switch(bool on) {
    {
        Settings s("car", true);
        s.SetBool("on", on);
    }
    vTaskDelay(pdMS_TO_TICKS(300));
    esp_restart();
}

void Run(RobotFaceDisplay* display) {
    auto& app = Application::GetInstance();
    display->SetCarMode(true);
    CarData d;
    InitWifi();
    int64_t last_alarm = 0;

    while (true) {
        d = CarData();
        d.state = CarData::kSearching;
        display->SetCarData(d);
        if (!JoinDongle()) {
            d.state = CarData::kNotFound;
            display->SetCarData(d);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        d.state = CarData::kConnecting;
        display->SetCarData(d);
        int s = ConnectElm();
        if (s < 0) {
            esp_wifi_disconnect();
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }
        Cmd(s, "ATZ", 3000);
        for (const char* c : {"ATE0", "ATL0", "ATS0", "ATH0", "ATSP0"}) Cmd(s, c);
        Cmd(s, "0100", 8000);  // protocol search can take a few seconds

        int loop = 0, fails = 0;
        while (fails < 5) {
            uint8_t b[8];
            bool any = false;
            if (Pid(s, "010D", b, 1) >= 1) { d.speed = b[0]; any = true; }
            if (Pid(s, "010C", b, 2) >= 2) { d.rpm = (b[0] * 256 + b[1]) / 4; any = true; }
            if (loop % 5 == 0) {
                if (Pid(s, "0105", b, 1) >= 1) d.coolant = b[0] - 40;
                std::string v = Cmd(s, "ATRV");
                float volt = strtof(v.c_str(), nullptr);
                if (volt > 5 && volt < 20) d.volt = volt;
            }
            if (loop % 30 == 0 && Pid(s, "0101", b, 4) >= 1) d.dtc = b[0] & 0x7F;
            fails = any ? 0 : fails + 1;
            d.state = CarData::kOk;
            display->SetCarData(d);

            // Spoken-free alarms: overheating or engine fault light.
            int64_t now = esp_timer_get_time() / 1000;
            if ((d.coolant >= 105 || (d.volt > 0 && d.volt < 11.5f)) && now - last_alarm > 15000) {
                last_alarm = now;
                app.PlaySound(Lang::Sounds::OGG_EXCLAMATION);
            }
            loop++;
            vTaskDelay(pdMS_TO_TICKS(150));
        }
        ESP_LOGW(TAG, "Lost the dongle, reconnecting");
        close(s);
        esp_wifi_disconnect();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

}  // namespace CarMode
