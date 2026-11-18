// microSD card media: MP3 music from /sdcard/music and a JPEG photo
// slideshow (with cross-fade) from /sdcard/photos. Both are driven by MCP
// tools, so the user can ask the assistant by voice.
#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

class RobotFaceDisplay;

class RobotMedia {
public:
    explicit RobotMedia(RobotFaceDisplay* display);
    bool Mounted() const { return mounted_; }

    void StopPhotos() { photos_on_ = false; }
    bool PhotosOn() const { return photos_on_; }
    // Voice commands for music, matched on the user's own words (not the AI).
    void OnUserText(const std::string& text);
    void StartMusic(std::vector<std::string> files);

    // Music player controls (used by the on-screen player).
    bool MusicOn() const { return music_on_; }
    bool Paused() const { return paused_; }
    void TogglePause() { paused_ = !paused_; }
    void Next() { paused_ = false; skip_ = true; }
    void Prev();
    void Stop() { music_on_ = false; skip_ = true; }
    void PlayAll();
    // Song names on the SD card, sorted; PlayFrom(i) plays them in order from i.
    std::vector<std::string> MusicNames();
    void PlayFrom(size_t i);
    float Progress() const { return progress_; }
    std::string NowPlaying();
    static std::vector<std::string> ListFiles(const char* dir, const char* const* exts) { return List(dir, exts); }

private:
    RobotFaceDisplay* display_;
    bool mounted_ = false;

    std::atomic<bool> music_on_{false};
    std::atomic<bool> skip_{false};
    std::atomic<bool> photos_on_{false};
    std::atomic<bool> paused_{false};
    std::atomic<float> progress_{0};
    std::mutex mu_;                     // guards queue_, now_playing_, current_, history_
    std::vector<std::string> queue_;  // full paths to play
    std::string now_playing_;
    std::string current_;
    std::vector<std::string> history_;
    bool going_back_ = false;

    void Mount();
    static std::vector<std::string> List(const char* dir, const char* const* exts);
    void AddTools();
    static void MusicTask(void* arg);
    bool PlayFile(const std::string& path);
    static void PhotoTask(void* arg);
    bool LoadPhoto(const std::string& path, uint16_t* out);
};
