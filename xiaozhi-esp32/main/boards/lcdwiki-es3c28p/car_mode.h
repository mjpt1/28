// Car mode: the robot joins the Wi-Fi of an ELM327 OBD-II dongle (v1.5,
// usually 192.168.0.10:35000) and shows a live dashboard. The dongle has no
// internet, so the voice assistant is off while car mode is on.
#pragma once

class RobotFaceDisplay;

namespace CarMode {

bool Enabled();
// Saves the choice and restarts the board.
void Switch(bool on);
// Runs forever: Wi-Fi to the dongle, OBD polling, dashboard updates.
[[noreturn]] void Run(RobotFaceDisplay* display);

}  // namespace CarMode
