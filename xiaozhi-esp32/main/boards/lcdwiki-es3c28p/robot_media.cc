#include "robot_media.h"

#include "application.h"
#include "mcp_server.h"
#include "robot_face.h"

#include <dirent.h>
#include <driver/sdmmc_host.h>
#include <esp_audio_simple_dec.h>
#include <decoder/impl/esp_mp3_dec.h>
#include <esp_audio_simple_dec_default.h>
#include <esp_heap_caps.h>
#include <esp_jpeg_dec.h>
#include <lvgl.h>
#include <esp_log.h>
#include <esp_random.h>
#include <esp_vfs_fat.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdmmc_cmd.h>
#include <sys/stat.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>

#define TAG "RobotMedia"

namespace {

constexpr const char* kMount = "/sdcard";
constexpr int kPhotoW = 320, kPhotoH = 240;
constexpr int kPhotoSeconds = 8;
// From LVGL's bundled lodepng (its header clashes in C++).
extern "C" unsigned lodepng_decode32(unsigned char** out, unsigned* w, unsigned* h, const unsigned char* in,
                                     size_t insize);
constexpr size_t kMaxJpeg = 6 * 1024 * 1024;

// LCDWiki ES3C28P microSD: SDIO, CLK IO38, CMD IO40, DATA0 IO39 (1-bit mode).
constexpr gpio_num_t kSdClk = GPIO_NUM_38, kSdCmd = GPIO_NUM_40, kSdD0 = GPIO_NUM_39;

bool HasExt(const std::string& name, const char* const* exts) {
    std::string n = name;
    std::transform(n.begin(), n.end(), n.begin(), ::tolower);
    for (const char* const* e = exts; *e; e++) {
        size_t l = strlen(*e);
        if (n.size() > l && n.compare(n.size() - l, l, *e) == 0) return true;
    }
    return false;
}

std::string BaseName(const std::string& path) {
    size_t s = path.rfind('/');
    std::string b = s == std::string::npos ? path : path.substr(s + 1);
    size_t d = b.rfind('.');
    return d == std::string::npos ? b : b.substr(0, d);
}

std::string Lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    return s;
}

const char* const kMusicExt[] = {".mp3", nullptr};
const char* const kPhotoExt[] = {".jpg", ".jpeg", ".png", nullptr};
}  // namespace

// Accept both folder names: /sdcard/photos and /sdcard/photo.
static std::vector<std::string> PhotoFiles();

namespace {

}  // namespace

RobotMedia::RobotMedia(RobotFaceDisplay* display) : display_(display) {
    Mount();
    esp_mp3_dec_register();  // the base decoder; simple_dec only adds the parser
    esp_audio_simple_dec_register_default();
    AddTools();
    xTaskCreatePinnedToCore(MusicTask, "music", 8192, this, 4, nullptr, 1);
    xTaskCreatePinnedToCore(PhotoTask, "photos", 6144, this, 2, nullptr, 1);
}

void RobotMedia::Mount() {
    esp_vfs_fat_sdmmc_mount_config_t mc = {};
    mc.format_if_mount_failed = false;
    mc.max_files = 4;
    mc.allocation_unit_size = 16 * 1024;
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 1;
    slot.clk = kSdClk;
    slot.cmd = kSdCmd;
    slot.d0 = kSdD0;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    sdmmc_card_t* card = nullptr;
    esp_err_t err = esp_vfs_fat_sdmmc_mount(kMount, &host, &slot, &mc, &card);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SD card not mounted: %s", esp_err_to_name(err));
        return;
    }
    mounted_ = true;
    ESP_LOGI(TAG, "SD card %s, %llu MB", card->cid.name,
             ((uint64_t)card->csd.capacity) * card->csd.sector_size / (1024 * 1024));
    ESP_LOGI(TAG, "music: %d files, photos: %d files",
             (int)List("/sdcard/music", kMusicExt).size(), (int)PhotoFiles().size());
}

std::vector<std::string> RobotMedia::List(const char* dir, const char* const* exts) {
    std::vector<std::string> out;
    DIR* d = opendir(dir);
    if (d == nullptr) return out;
    while (struct dirent* e = readdir(d)) {
        if (e->d_name[0] == '.') continue;
        std::string name = e->d_name;
        std::string full = std::string(dir) + "/" + name;
        if (e->d_type == DT_DIR) {  // look inside subfolders too
            auto sub = List(full.c_str(), exts);
            out.insert(out.end(), sub.begin(), sub.end());
        } else if (HasExt(name, exts)) {
            out.push_back(full);
        }
    }
    closedir(d);
    std::sort(out.begin(), out.end());
    return out;
}

void RobotMedia::StartMusic(std::vector<std::string> files) {
    music_on_ = false;
    skip_ = true;
    vTaskDelay(pdMS_TO_TICKS(300));
    paused_ = false;
    std::lock_guard<std::mutex> lk(mu_);
    queue_ = std::move(files);
    music_on_ = !queue_.empty();
}

void RobotMedia::PlayAll() {
    auto files = List("/sdcard/music", kMusicExt);
    for (size_t i = files.size(); i > 1; i--) std::swap(files[i - 1], files[esp_random() % i]);
    StartMusic(std::move(files));
}

std::vector<std::string> RobotMedia::MusicNames() {
    auto files = List("/sdcard/music", kMusicExt);
    for (auto& f : files) f = BaseName(f);
    return files;
}

void RobotMedia::PlayFrom(size_t i) {
    auto files = List("/sdcard/music", kMusicExt);
    if (i >= files.size()) return;
    std::rotate(files.begin(), files.begin() + i, files.end());
    StartMusic(std::move(files));
}

void RobotMedia::Prev() {
    std::lock_guard<std::mutex> lk(mu_);
    if (current_.empty()) return;
    queue_.insert(queue_.begin(), current_);  // the song after the previous one
    if (!history_.empty()) {
        queue_.insert(queue_.begin(), history_.back());
        history_.pop_back();
    }
    going_back_ = true;
    paused_ = false;
    skip_ = true;
}

std::string RobotMedia::NowPlaying() {
    std::lock_guard<std::mutex> lk(mu_);
    return now_playing_;
}

void RobotMedia::OnUserText(const std::string& t) {
    auto has = [&](std::initializer_list<const char*> words) {
        for (const char* w : words) if (t.find(w) != std::string::npos) return true;
        return false;
    };
    if (!mounted_ || !has({"آهنگ", "اهنگ", "موزیک", "موسیقی", "ترانه"})) return;
    if (has({"قطع", "بسه", "خاموش", "نگه دار", "متوقف", "تموم"})) {
        music_on_ = false;
        skip_ = true;
    } else if (has({"بعدی", "عوض"})) {
        skip_ = true;
    } else if (has({"پخش", "بذار", "بزار", "بزن", "بخون", "بگذار"})) {
        ESP_LOGI(TAG, "Voice command: play music");
        PlayAll();
    }
}

static std::vector<std::string> PhotoFiles() {
    auto a = RobotMedia::ListFiles("/sdcard/photos", kPhotoExt);
    auto b = RobotMedia::ListFiles("/sdcard/photo", kPhotoExt);
    a.insert(a.end(), b.begin(), b.end());
    return a;
}

// ---------------------------------------------------------------------------
// Music
// ---------------------------------------------------------------------------
void RobotMedia::MusicTask(void* arg) {
    auto* self = static_cast<RobotMedia*>(arg);
    vTaskDelay(pdMS_TO_TICKS(12000));  // report after USB serial is back
    for (const char* dir : {"/sdcard", "/sdcard/photos"}) {  // show what the card holds
        if (DIR* d = opendir(dir)) {
            int n = 0;
            while (struct dirent* e = readdir(d)) {
                if (n++ < 15) ESP_LOGI(TAG, "%s: %s%s", dir, e->d_name, e->d_type == DT_DIR ? "/" : "");
            }
            closedir(d);
        } else {
            ESP_LOGW(TAG, "%s: cannot open", dir);
        }
    }
    ESP_LOGI(TAG, "SD %s, music %d, photos %d", self->mounted_ ? "mounted" : "NOT mounted",
             (int)List("/sdcard/music", kMusicExt).size(), (int)PhotoFiles().size());
    while (true) {
        if (!self->music_on_ || self->queue_.empty()) {
            self->music_on_ = false;
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        std::string path;
        {
            std::lock_guard<std::mutex> lk(self->mu_);
            if (self->queue_.empty()) continue;
            path = self->queue_.front();
            self->queue_.erase(self->queue_.begin());
            self->now_playing_ = BaseName(path);
            self->current_ = path;
        }
        ESP_LOGI(TAG, "Playing %s", path.c_str());
        self->progress_ = 0;
        self->PlayFile(path);
        {
            std::lock_guard<std::mutex> lk(self->mu_);
            if (!self->going_back_) self->history_.push_back(path);
            if (self->history_.size() > 50) self->history_.erase(self->history_.begin());
            self->going_back_ = false;
            self->now_playing_.clear();
        }
    }
}

bool RobotMedia::PlayFile(const std::string& path) {
    FILE* f = fopen(path.c_str(), "rb");
    if (f == nullptr) return false;
    esp_audio_simple_dec_cfg_t cfg = {};
    cfg.dec_type = ESP_AUDIO_SIMPLE_DEC_TYPE_MP3;
    esp_audio_simple_dec_handle_t dec = nullptr;
    if (esp_audio_simple_dec_open(&cfg, &dec) != ESP_AUDIO_ERR_OK) {
        fclose(f);
        return false;
    }
    auto& app = Application::GetInstance();
    auto& audio = app.GetAudioService();
    const int out_rate = audio.OutputSampleRate();

    const size_t kIn = 4096;
    uint8_t* in = (uint8_t*)heap_caps_malloc(kIn, MALLOC_CAP_SPIRAM);
    uint32_t out_cap = 4608 * 2;
    uint8_t* out = (uint8_t*)heap_caps_malloc(out_cap, MALLOC_CAP_SPIRAM);
    double pos = 0;  // resampler phase
    skip_ = false;
    fseek(f, 0, SEEK_END);
    long fsize = std::max(1L, ftell(f));
    fseek(f, 0, SEEK_SET);
    int listen_ms = 0;

    while (music_on_ && !skip_) {
        // The assistant is talking or listening: wait, then continue the song.
        if (paused_) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        auto state = app.GetDeviceState();
        if (state != kDeviceStateIdle) {
            // After a reply the robot keeps listening; end that so the song goes on.
            listen_ms = state == kDeviceStateListening ? listen_ms + 200 : 0;
            if (listen_ms >= 8000) {
                listen_ms = 0;
                app.Schedule([&app]() {
                    if (app.GetDeviceState() == kDeviceStateListening) app.ToggleChatState();
                });
            }
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        listen_ms = 0;
        size_t n = fread(in, 1, kIn, f);
        progress_ = (float)ftell(f) / fsize;
        esp_audio_simple_dec_raw_t raw = {};
        raw.buffer = in;
        raw.len = n;
        raw.eos = n < kIn;
        while (raw.len > 0 || raw.eos) {
            esp_audio_simple_dec_out_t frame = {};
            frame.buffer = out;
            frame.len = out_cap;
            esp_audio_err_t r = esp_audio_simple_dec_process(dec, &raw, &frame);
            if (r == ESP_AUDIO_ERR_BUFF_NOT_ENOUGH) {
                out_cap = frame.needed_size;
                out = (uint8_t*)heap_caps_realloc(out, out_cap, MALLOC_CAP_SPIRAM);
                continue;
            }
            if (r != ESP_AUDIO_ERR_OK) break;
            if (frame.decoded_size > 0) {
                esp_audio_simple_dec_info_t info = {};
                esp_audio_simple_dec_get_info(dec, &info);
                int ch = std::max<int>(1, info.channel);
                int rate = info.sample_rate ? info.sample_rate : 44100;
                const int16_t* s = (const int16_t*)out;
                int frames = frame.decoded_size / 2 / ch;
                // Mono mix and linear resample to the codec rate.
                double step = (double)rate / out_rate;
                std::vector<int16_t> pcm;
                pcm.reserve(frames * out_rate / rate + 2);
                while (pos < frames - 1) {
                    int i = (int)pos;
                    float t = pos - i;
                    int a = 0, b = 0;
                    for (int c = 0; c < ch; c++) {
                        a += s[i * ch + c];
                        b += s[(i + 1) * ch + c];
                    }
                    pcm.push_back((int16_t)((a + (b - a) * t) / ch));
                    pos += step;
                }
                pos -= frames;
                if (!pcm.empty()) audio.PushPcmToPlayback(std::move(pcm), 2000);
            }
            raw.buffer += raw.consumed;
            raw.len -= raw.consumed;
            if (raw.consumed == 0 && frame.decoded_size == 0) break;
            if (raw.len == 0 && raw.eos) break;
        }
        if (n < kIn) break;  // end of file
    }
    heap_caps_free(in);
    heap_caps_free(out);
    esp_audio_simple_dec_close(dec);
    fclose(f);
    return true;
}

// ---------------------------------------------------------------------------
// Photos: decode with a power-of-two downscale, fit into 320x240, cross-fade.
// ---------------------------------------------------------------------------
bool RobotMedia::LoadPhoto(const std::string& path, uint16_t* dst) {
    struct stat st;
    if (stat(path.c_str(), &st) != 0 || st.st_size <= 0 || (size_t)st.st_size > kMaxJpeg) {
        ESP_LOGW(TAG, "Skip %s (size)", path.c_str());
        return false;
    }
    FILE* f = fopen(path.c_str(), "rb");
    if (f == nullptr) return false;
    uint8_t* data = (uint8_t*)heap_caps_malloc(st.st_size, MALLOC_CAP_SPIRAM);
    if (data == nullptr) {
        fclose(f);
        return false;
    }
    size_t got = fread(data, 1, st.st_size, f);
    fclose(f);

    bool ok = false;
    uint8_t* img = nullptr;
    int iw = 0, ih = 0;

    if (got > 8 && memcmp(data, "\x89PNG", 4) == 0) {
        // PNG decodes to full-size RGBA, so cap the pixel count to fit PSRAM.
        unsigned w = 0, h = 0;
        auto be32 = [&](int o) { return (unsigned)data[o] << 24 | data[o + 1] << 16 | data[o + 2] << 8 | data[o + 3]; };
        w = got > 24 ? be32(16) : 0;  // IHDR width/height
        h = got > 24 ? be32(20) : 0;
        bool fits = w > 0 && h > 0 && (size_t)w * h <= 1200000;
        unsigned char* rgba = nullptr;
        if (fits && lodepng_decode32(&rgba, &w, &h, data, got) == 0) {
            img = (uint8_t*)jpeg_calloc_align((size_t)w * h * 2, 16);
            if (img != nullptr) {
                uint16_t* px = (uint16_t*)img;
                for (size_t i = 0; i < (size_t)w * h; i++) {
                    const unsigned char* c = rgba + i * 4;
                    unsigned a = c[3];  // blend onto black
                    unsigned r = c[0] * a / 255, g = c[1] * a / 255, b = c[2] * a / 255;
                    px[i] = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
                }
                iw = w;
                ih = h;
                ok = true;
            }
        } else {
            ESP_LOGW(TAG, "Skip %s (PNG too big: %ux%u, max ~1.2 MP)", path.c_str(), w, h);
        }
        if (rgba) lv_free(rgba);
        heap_caps_free(data);
        if (!ok) return false;
    } else {
    jpeg_dec_handle_t dec = nullptr;
    jpeg_dec_config_t cfg = DEFAULT_JPEG_DEC_CONFIG();
    cfg.output_type = JPEG_PIXEL_FORMAT_RGB565_LE;
    jpeg_dec_io_t io = {};
    jpeg_dec_header_info_t hdr = {};

    if (jpeg_dec_open(&cfg, &dec) == JPEG_ERR_OK) {
        io.inbuf = data;
        io.inbuf_len = got;
        if (jpeg_dec_parse_header(dec, &io, &hdr) == JPEG_ERR_OK) {
            jpeg_dec_close(dec);
            dec = nullptr;
            // Smallest power-of-two scale (max 1/8) that still covers the screen.
            int k = 1;
            while (k < 8 && hdr.width / (k * 2) >= kPhotoW && hdr.height / (k * 2) >= kPhotoH) k *= 2;
            if (k > 1) {
                cfg.scale.width = (hdr.width / k) & ~7;
                cfg.scale.height = (hdr.height / k) & ~7;
            }
            iw = k > 1 ? cfg.scale.width : hdr.width;
            ih = k > 1 ? cfg.scale.height : hdr.height;
            if (jpeg_dec_open(&cfg, &dec) == JPEG_ERR_OK) {
                io = {};
                io.inbuf = data;
                io.inbuf_len = got;
                jpeg_dec_parse_header(dec, &io, &hdr);
                int len = 0;
                jpeg_dec_get_outbuf_len(dec, &len);
                img = (uint8_t*)jpeg_calloc_align(len, 16);
                if (img != nullptr) {
                    io.outbuf = img;
                    ok = jpeg_dec_process(dec, &io) == JPEG_ERR_OK;
                }
            }
        }
    }
    if (dec) jpeg_dec_close(dec);
    heap_caps_free(data);
    if (!ok) {
        ESP_LOGW(TAG, "Cannot decode %s (progressive JPEG?)", path.c_str());
        if (img) jpeg_free_align(img);
        return false;
    }
    }

    // Fit inside 320x240 keeping the aspect ratio; black bars around it.
    const uint16_t* src = (const uint16_t*)img;
    float sc = std::min((float)kPhotoW / iw, (float)kPhotoH / ih);
    int ow = std::max(1, (int)(iw * sc)), oh = std::max(1, (int)(ih * sc));
    int x0 = (kPhotoW - ow) / 2, y0 = (kPhotoH - oh) / 2;
    std::fill(dst, dst + kPhotoW * kPhotoH, 0);
    for (int y = 0; y < oh; y++) {
        int sy = std::min(ih - 1, (int)(y / sc));
        for (int x = 0; x < ow; x++) {
            int sx = std::min(iw - 1, (int)(x / sc));
            dst[(y0 + y) * kPhotoW + x0 + x] = src[sy * iw + sx];
        }
    }
    jpeg_free_align(img);
    return true;
}

void RobotMedia::PhotoTask(void* arg) {
    auto* self = static_cast<RobotMedia*>(arg);
    uint16_t* buf = (uint16_t*)heap_caps_malloc(kPhotoW * kPhotoH * 2, MALLOC_CAP_SPIRAM);
    while (true) {
        if (!self->photos_on_) {
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }
        auto files = PhotoFiles();
        if (files.empty()) {
            self->photos_on_ = false;
            continue;
        }
        self->display_->SetPhotoMode(true);
        bool shown = false;
        for (size_t i = 0; i < files.size() && self->photos_on_; i++) {
            if (!self->LoadPhoto(files[i], buf)) continue;
            shown = true;
            self->display_->ShowPhoto(buf);
            for (int t = 0; t < kPhotoSeconds * 10 && self->photos_on_; t++) vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (!shown) self->photos_on_ = false;
        if (!self->photos_on_) self->display_->SetPhotoMode(false);
    }
}

// ---------------------------------------------------------------------------
void RobotMedia::AddTools() {
    auto& mcp = McpServer::GetInstance();

    mcp.AddTool("self.music.list", "List the songs on the robot's memory card (folder music).",
                PropertyList(), [this](const PropertyList&) -> ToolResult {
                    if (!mounted_) return std::unexpected(std::string("No memory card"));
                    auto files = List("/sdcard/music", kMusicExt);
                    if (files.empty()) return std::string("The music folder is empty.");
                    std::string out;
                    for (auto& f : files) out += BaseName(f) + "\n";
                    return out;
                });

    mcp.AddTool("self.music.play",
                "Play songs from the robot's own memory card. Call this ONLY when the user clearly asks for music or a song in this turn; never for other requests. Use this (not online music search) "
                "when the user asks to play music or a song. query: part of a song or artist name "
                "in Latin letters as in the file names; leave empty for a general request like 'play music'. shuffle: random order.",
                PropertyList({Property("query", kPropertyTypeString, std::string("")),
                              Property("shuffle", kPropertyTypeBoolean, true)}),
                [this](const PropertyList& p) -> ToolResult {
                    if (!mounted_) return std::unexpected(std::string("No memory card"));
                    auto files = List("/sdcard/music", kMusicExt);
                    std::string q = Lower(p["query"].value<std::string>());
                    std::vector<std::string> pick;
                    for (auto& f : files) {
                        if (q.empty() || Lower(BaseName(f)).find(q) != std::string::npos) pick.push_back(f);
                    }
                    if (pick.empty()) pick = files;  // names are Latin; a Persian query rarely matches
                    if (pick.empty()) return std::unexpected(std::string("The music folder is empty."));
                    if (p["shuffle"].value<bool>()) {
                        for (size_t i = pick.size(); i > 1; i--) std::swap(pick[i - 1], pick[esp_random() % i]);
                    }
                    StartMusic(pick);
                    std::string first = BaseName(pick[0]);
                    return "Playing " + std::to_string(pick.size()) + " song(s), first: " + first +
                           ". Music starts when you stop talking.";
                });

    mcp.AddTool("self.music.next", "Skip to the next song.", PropertyList(),
                [this](const PropertyList&) -> ToolResult {
                    skip_ = true;
                    return true;
                });

    mcp.AddTool("self.music.stop", "Stop the music.", PropertyList(),
                [this](const PropertyList&) -> ToolResult {
                    Stop();
                    return true;
                });

    mcp.AddTool("self.music.now_playing", "Name of the song that is playing now.", PropertyList(),
                [this](const PropertyList&) -> ToolResult {
                    auto name = NowPlaying();
                    return name.empty() ? std::string("Nothing is playing.") : name;
                });

    mcp.AddTool("self.photos.show",
                "Show the photos from the robot's memory card one by one as a slideshow with a "
                "fade effect. Touching the screen stops it.",
                PropertyList(), [this](const PropertyList&) -> ToolResult {
                    if (!mounted_) return std::unexpected(std::string("No memory card"));
                    if (PhotoFiles().empty()) {
                        return std::unexpected(std::string("No JPG/PNG photos in the photos folder"));
                    }
                    photos_on_ = true;
                    return true;
                });

    mcp.AddTool("self.photos.stop", "Stop the photo slideshow.", PropertyList(),
                [this](const PropertyList&) -> ToolResult {
                    photos_on_ = false;
                    return true;
                });
}
