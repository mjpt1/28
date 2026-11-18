// Extra abilities of the robot, exposed to the assistant as MCP tools:
//   - alarms (one-shot or daily), timers and reminders, kept in NVS
//   - room temperature / humidity from an optional SHT3x / SHT4x on I2C
//   - phone messages both ways through a Bale bot
#pragma once

#include <driver/i2c_master.h>

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

class RobotFaceDisplay;

class RobotSkills {
public:
    RobotSkills(i2c_master_bus_handle_t i2c, RobotFaceDisplay* display);
    void Start();

    // Touch while an alarm rings stops it. Returns true if it was ringing.
    bool StopRinging();
    // Sends a text to the user's phone through the Bale bot.
    bool Notify(const std::string& msg);
    // Code to send to the Bale bot to link the phone; 0 once linked.
    int PairCode() const { return chat_id_ == 0 ? pair_code_ : 0; }

private:
    int pair_code_ = 0;
    struct Event {
        int id;
        int64_t at;  // UTC seconds
        bool daily;
        std::string label;
    };

    i2c_master_bus_handle_t i2c_;
    RobotFaceDisplay* display_;
    std::mutex mutex_;
    std::vector<Event> events_;
    int next_id_ = 1;
    volatile bool ringing_ = false;

    i2c_master_dev_handle_t sht_ = nullptr;
    bool sht4x_ = false;

    std::string token_;
    int64_t chat_id_ = 0;
    int64_t update_offset_ = 0;
    std::vector<std::string> inbox_;

    void Load();
    void Save();
    void AddTools();
    int AddEvent(int64_t at, bool daily, const std::string& label);
    std::string ListEvents();
    void Fire(const Event& e);
    void ProbeSensor();
    bool ReadClimate(float& t, float& rh);
    std::string BaleCall(const std::string& method, const std::string& json);
    void PollPhone();
    static void TaskMain(void* arg);
};
