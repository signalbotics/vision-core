#ifndef NREAL_HPP
#define NREAL_HPP

#include <iostream>
#include <string>
#include <vector>
#include <optional>
#include <deque>
#include <cstdint>
#include <thread>
#include <chrono>
#include <hidapi/hidapi.h>
#include <eigen3/Eigen/Dense>
#include "../../common/json.hpp"

using json = nlohmann::json;
using namespace std;
using namespace std::chrono;
using namespace Eigen;

uint32_t crc32_adler(const std::vector<uint8_t>& buf);

struct Packet {
    uint8_t category;
    uint8_t cmd_id;
    vector<uint8_t> data;

    static std::optional<Packet> deserialize(const std::vector<uint8_t>& data);
    static std::vector<std::vector<uint8_t>> split(const std::vector<uint8_t>& data, char delimiter);
    vector<uint8_t> serialize() const;
};

enum class Error {
    None,
    PacketTimeout,
    Other
};

struct Gyroscope {
    float x, y, z;
};

struct Accelerometer {
    float x, y, z;
};

enum DisplayMode {
    SameOnBoth,
    HalfSBS,
    Stereo,
    HighRefreshRate,
    HighRefreshRateSBS,
    Unsupported
};

class NrealLight {
    hid_device* mcu_device;
    hid_device* ov580_device;
    deque<Packet> pending_packets;
    steady_clock::time_point last_heartbeat;
    json config_json;
    Vector3f gyro_bias = Vector3f::Zero();
    Vector3f accelerometer_bias = Vector3f::Zero();

    static constexpr uint16_t MCU_VID = 0x0486;
    static constexpr uint16_t MCU_PID = 0x573c;
    static constexpr uint16_t OV580_VID = 0x05A9;
    static constexpr uint16_t OV580_PID = 0x0680;
    static constexpr int OV_580_TIMEOUT = 250;

    static constexpr double DISPLAY_TILT = -0.265;
    static constexpr double DISPLAY_DIVERGENCE = 0.02;

public:
    NrealLight();
    ~NrealLight();
    void sendHeartbeatIfNeeded();
    std::vector<uint8_t> command(uint8_t cmd, uint8_t subcmd);
    bool parseConfigData(const std::vector<uint8_t>& config_data);
    bool parseIMUPacket(const vector<uint8_t>& packet, Gyroscope& gyro, Accelerometer& acc);
    Error run_command(const Packet& command, hid_device* device);
    optional<Packet> readPacket(hid_device* device, int timeout = 250);
    std::optional<std::pair<Gyroscope, Accelerometer>> readIMUData();
    DisplayMode getDisplayMode();
    Error setDisplayMode(DisplayMode display_mode);
    bool readConfigData();
};

#endif // NREAL_HPP
