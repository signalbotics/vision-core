#include "nreal_light.hpp"

using json = nlohmann::json;
using namespace std;
using namespace std::chrono;
using namespace Eigen;

uint32_t crc32_adler(const std::vector<uint8_t>& buf) {
    const uint32_t CRCTABLE[256] = {
        0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535,
        0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd,
        0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d,
        0x6ddde4eb, 0xf4d4b551, 0x83d385c7, 0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec,
        0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4,
        0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
        0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59, 0x26d930ac,
        0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
        0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab,
        0xb6662d3d, 0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f,
        0x9fbfe4a5, 0xe8b8d433, 0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb,
        0x086d3d2d, 0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
        0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea,
        0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65, 0x4db26158, 0x3ab551ce,
        0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a,
        0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
        0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409,
        0xce61e49f, 0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
        0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739,
        0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8,
        0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1, 0xf00f9344, 0x8708a3d2, 0x1e01f268,
        0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0,
        0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8,
        0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
        0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef,
        0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703,
        0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7,
        0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d, 0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a,
        0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae,
        0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
        0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777, 0x88085ae6,
        0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
        0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d,
        0x3e6e77db, 0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5,
        0x47b2cf7f, 0x30b5ffe9, 0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605,
        0xcdd70693, 0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
        0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d,
    };
    uint32_t r = 0xffffffff;
    for (auto byte : buf) {
        uint8_t idx = byte ^ (r & 0xff);
        r = (r >> 8) ^ CRCTABLE[idx];
    }
    return r ^ 0xffffffff;
}

// struct Packet::Packet {
//     uint8_t category;
//     uint8_t cmd_id;
//     vector<uint8_t> data;

// }

std::optional<Packet> Packet::deserialize(const std::vector<uint8_t>& data) {
    if (data[0] != 2) {
        return std::nullopt; // Invalid packet, return None
    }
    auto end_it = std::find(data.begin(), data.end(), 3);
    if (end_it == data.end()) {
        return std::nullopt; // End delimiter not found, return None
    }

    std::vector<uint8_t> inner(data.begin() + 1, end_it);
    std::vector<std::vector<uint8_t>> parts = split(inner, ':');
    if (parts.size() < 3) {
        return std::nullopt; // Not enough parts, return None
    }

    Packet packet;
    packet.category = parts[1][0];
    packet.cmd_id = parts[2][0];
    packet.data = std::vector<uint8_t>(parts[3].begin(), parts[3].end());

    return packet;
}

std::vector<std::vector<uint8_t>> Packet::split(const std::vector<uint8_t>& data, char delimiter) {
    std::vector<std::vector<uint8_t>> result;
    size_t start = 0;
    auto it = std::find(data.begin(), data.end(), delimiter);
    while (it != data.end()) {
        result.emplace_back(data.begin() + start, it);
        start = std::distance(data.begin(), it) + 1;
        it = std::find(data.begin() + start, data.end(), delimiter);
    }
    if (start < data.size()) {
        result.emplace_back(data.begin() + start, data.end());
    }
    return result;
}


vector<uint8_t> Packet::serialize() const {
    vector<uint8_t> result = {2, ':', category, ':', cmd_id, ':'};
    result.insert(result.end(), data.begin(), data.end());
    result.insert(result.end(), {':', '0', ':'});
    uint32_t crc = crc32_adler(result);
    char crcStr[9];
    snprintf(crcStr, sizeof(crcStr), "%08x", crc);
    result.insert(result.end(), crcStr, crcStr + 8);
    result.push_back(':');
    result.push_back(3);
    return result;
}



NrealLight::NrealLight() {
    hid_init();

    mcu_device = hid_open(MCU_VID, MCU_PID, nullptr);
    if (!mcu_device) {
        cerr << "Failed to open MCU device." << endl;
        exit(1); // or handle error appropriately
    }

    ov580_device = hid_open(OV580_VID, OV580_PID, nullptr);
    if (!ov580_device) {
        cerr << "Failed to open OV580 device." << endl;
        exit(1); // or handle error appropriately
    }
    last_heartbeat = steady_clock::now();
    // Send initialization commands using run_command
    run_command(Packet{ '@', '3', {'1'} }, mcu_device); // Send "Yes, I am a working SDK" command
    run_command(Packet{ '1', 'L', {'1'} }, mcu_device); // Enable Ambient Light event
    run_command(Packet{ '1', 'N', {'1'} }, mcu_device); // Enable VSync event
    
    // Turn off IMU stream for configuration
    std::vector<uint8_t> command = {2, 0x19, 0x0, 0, 0, 0, 0};
    if (hid_write(ov580_device, command.data(), command.size()) == -1) {
        cerr << "Failed to send command to turn off IMU stream." << endl;
        exit(1);
    }

    // // Read and parse configuration data
    // if (!readConfigData()) {
    //     cerr << "Failed to read and parse configuration data." << endl;
    //     exit(1);
    // }

    // Turn on IMU stream
    command = {2, 0x19, 0x1, 0, 0, 0, 0};
    if (hid_write(ov580_device, command.data(), command.size()) == -1) {
        cerr << "Failed to send command to turn on IMU stream." << endl;
        exit(1);
    }
}

NrealLight::~NrealLight() {
    if (mcu_device) {
        hid_close(mcu_device);
        mcu_device = nullptr;
    }
    if (ov580_device) {
        hid_close(ov580_device);
        ov580_device = nullptr;
    }
    hid_exit(); // Ensure global cleanup
}

DisplayMode NrealLight::getDisplayMode() {
    Packet command = { '3', '3', {'x'} };
    if (run_command(command, mcu_device) != Error::None) {
        cerr << "Failed to get display mode." << endl;
        return DisplayMode::Unsupported;
    }

    auto packet = readPacket(mcu_device, OV_580_TIMEOUT);
    if (!packet || packet->data.empty()) {
        cerr << "Failed to read display mode response." << endl;
        return DisplayMode::Unsupported;
    }

    cout << "Display mode packet data: " << packet->data[0] << endl; // Debugging statement

    switch (packet->data[0]) {
        case '1': return DisplayMode::SameOnBoth;
        case '2': return DisplayMode::HalfSBS;
        case '3': return DisplayMode::Stereo;
        case '4': return DisplayMode::HighRefreshRate;
        default: return DisplayMode::Unsupported;
    } 
}

Error NrealLight::setDisplayMode(DisplayMode displayMode) {
    uint8_t display_mode_byte;
    switch (displayMode) {
        case DisplayMode::SameOnBoth:
            display_mode_byte = '1';
            break;
        case DisplayMode::HalfSBS:
            display_mode_byte = '2';
            break;
        case DisplayMode::Stereo:
            display_mode_byte = '3';
            break;
        case DisplayMode::HighRefreshRate:
            display_mode_byte = '4';
            break;
        default:
            cerr << "Unsupported display mode." << endl;
            return Error::Other;
    }

    Packet command = { '1', '3', { display_mode_byte } };
    if (run_command(command, mcu_device) != Error::None) {
        cerr << "Failed to set display mode." << endl;
        return Error::Other;
    }

    auto packet = readPacket(mcu_device, OV_580_TIMEOUT);
    if (!packet || packet->data.empty() || packet->data[0] != display_mode_byte) {
        cerr << "Display mode setting unsuccessful." << endl;
        return Error::Other;
    }

    cout << "Display mode set packet data: " << packet->data[0] << endl; // Debugging statement

    return Error::None;
}


void NrealLight::sendHeartbeatIfNeeded() {
    auto now = steady_clock::now();
    if (duration_cast<milliseconds>(now - last_heartbeat) > milliseconds(250)) {
        Packet packet = { '@', 'K', {'x'} };
        auto serialized = packet.serialize();
        hid_write(mcu_device, &serialized[0], serialized.size());
        last_heartbeat = now;
    }
}

std::vector<uint8_t> NrealLight::command(uint8_t cmd, uint8_t subcmd) {
    std::vector<uint8_t> packet = {2, cmd, subcmd, 0, 0, 0, 0};
    if (hid_write(ov580_device, packet.data(), packet.size()) == -1) {
        throw std::runtime_error("Command write failed");
    }
    for (int i = 0; i < 64; ++i) {
        std::vector<uint8_t> result(128);
        int size = hid_read_timeout(ov580_device, result.data(), result.size(), OV_580_TIMEOUT);
        if (size <= 0) throw std::runtime_error("Packet timeout");
        result.resize(size);
        if (result[0] == 2) {
            return result;
        }
    }
    throw std::runtime_error("Couldn't get acknowledgement to command");
}

Error NrealLight::run_command(const Packet& command, hid_device* device) {
    auto serialized = command.serialize();
    if (hid_write(device, &serialized[0], serialized.size()) == -1) {
        return Error::Other;
    }

    for (int i = 0; i < 64; ++i) {

        auto packet = readPacket(device, OV_580_TIMEOUT);
        if (!packet) return Error::PacketTimeout;
        if (packet->category == command.category + 1 && packet->cmd_id == command.cmd_id) {
            return Error::None;
        }
        pending_packets.push_back(*packet);
        std::cout << "Pending packets: " << pending_packets.size() << std::endl;
    }
    return Error::Other;
}

optional<Packet> NrealLight::readPacket(hid_device* device, int timeout) {
    vector<uint8_t> buffer(64);
    int size = hid_read_timeout(device, buffer.data(), buffer.size(), timeout);
    if (size <= 0) return nullopt;
    return Packet::deserialize(buffer);
}

std::optional<std::pair<Gyroscope, Accelerometer>> NrealLight::readIMUData() {
    std::vector<uint8_t> packet_data(128);
    int bytes_read = hid_read_timeout(ov580_device, packet_data.data(), packet_data.size(), OV_580_TIMEOUT);
    if (bytes_read > 0) {
        Gyroscope gyro;
        Accelerometer acc;
        if (parseIMUPacket(packet_data, gyro, acc)) {
            return std::make_pair(gyro, acc);
        }
    }
    return std::nullopt;
}


bool NrealLight::parseIMUPacket(const std::vector<uint8_t>& packet, Gyroscope& gyro, Accelerometer& acc) {
    if (packet.size() < 64) return false;

    auto read_u64 = [&](int offset) {
        uint64_t value;
        memcpy(&value, &packet[offset], sizeof(value));
        return value;
    };

    auto read_i32 = [&](int offset) {
        int32_t value;
        memcpy(&value, &packet[offset], sizeof(value));
        return value;
    };

    auto read_u32 = [&](int offset) {
        uint32_t value;
        memcpy(&value, &packet[offset], sizeof(value));
        return value;
    };

    uint64_t gyro_timestamp = read_u64(44) / 1000;
    float gyro_mul = static_cast<float>(read_u32(52));
    float gyro_div = static_cast<float>(read_u32(56));
    int32_t gyro_x_raw = read_i32(60);
    int32_t gyro_y_raw = read_i32(64);
    int32_t gyro_z_raw = read_i32(68);

    gyro.x = (gyro_x_raw * gyro_mul / gyro_div) * M_PI / 180.0; // converting to radians
    gyro.y = (gyro_y_raw * gyro_mul / gyro_div) * M_PI / 180.0;
    gyro.z = (gyro_z_raw * gyro_mul / gyro_div) * M_PI / 180.0;

    uint64_t acc_timestamp = read_u64(72) / 1000;
    float acc_mul = static_cast<float>(read_u32(80));
    float acc_div = static_cast<float>(read_u32(84));
    int32_t acc_x_raw = read_i32(88);
    int32_t acc_y_raw = read_i32(92);
    int32_t acc_z_raw = read_i32(96);

    acc.x = (acc_x_raw * acc_mul / acc_div) * 9.81;
    acc.y = (acc_y_raw * acc_mul / acc_div) * 9.81;
    acc.z = (acc_z_raw * acc_mul / acc_div) * 9.81;

    if (std::isnan(gyro.x) || std::isnan(gyro.y) || std::isnan(gyro.z) ||
        std::isnan(acc.x) || std::isnan(acc.y) || std::isnan(acc.z)) {
        std::cerr << "Parsed NaN values from IMU packet" << std::endl;
        return false;
    }

    return true;
}

bool NrealLight::readConfigData() {
    // Command to start reading config (0x14)
    std::vector<uint8_t> start_command = {0x02, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (hid_write(ov580_device, start_command.data(), start_command.size()) == -1) {
        cerr << "Failed to send start command for configuration data." << endl;
        return false;
    }

    std::vector<uint8_t> config_data;
    while (true) {
        // Command to read next chunk of config (0x15)
        std::vector<uint8_t> read_command = {0x02, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00};
        if (hid_write(ov580_device, read_command.data(), read_command.size()) == -1) {
            cerr << "Failed to send read command for configuration data." << endl;
            return false;
        }

        std::vector<uint8_t> buffer(128);
        int bytes_read = hid_read_timeout(ov580_device, buffer.data(), buffer.size(), OV_580_TIMEOUT);
        if (bytes_read <= 0) {
            cerr << "Failed to read configuration data or timeout." << endl;
            return false;
        }

        // Log each chunk for debugging
        std::cout << "Read config chunk:";
        for (int i = 0; i < bytes_read; ++i) {
            printf(" %02x", buffer[i]);
        }
        std::cout << std::endl;

        // Append the chunk to config_data
        config_data.insert(config_data.end(), buffer.begin(), buffer.begin() + bytes_read);

        // Check for termination condition: start of new packet
        if (buffer[0] != 0x02 || buffer[1] != 0x01) { // Adjust the condition based on actual termination
            break;
        }
    }

    // Log the complete raw configuration data for debugging
    std::cout << "Complete config data:";
    for (size_t i = 0; i < config_data.size(); ++i) {
        printf(" %02x", config_data[i]);
    }
    std::cout << std::endl;

    return parseConfigData(config_data);
}



bool NrealLight::parseConfigData(const std::vector<uint8_t>& config_data) {
    std::string config_str(config_data.begin(), config_data.end());
    
    // Search for the JSON start token
    size_t json_start_pos = config_str.find("\n\n{");
    if (json_start_pos == std::string::npos) {
        cerr << "Invalid glasses config format (no start token)" << endl;
        return false;
    }

    // Extract the JSON string
    size_t json_end_pos = config_str.find("\n\n", json_start_pos + 2);
    if (json_end_pos == std::string::npos) {
        cerr << "Invalid glasses config format (no end token)" << endl;
        return false;
    }

    std::string json_str = config_str.substr(json_start_pos + 2, json_end_pos - (json_start_pos + 2));
    
    // Log the JSON string for debugging
    std::cout << "Extracted JSON string: " << json_str << std::endl;

    try {
        config_json = json::parse(json_str);
    } catch (json::parse_error& e) {
        cerr << "JSON parse error: " << e.what() << endl;
        return false;
    }

    // Parse bias values if present
    try {
        auto imu_config = config_json["IMU"]["device_1"];
        gyro_bias = Vector3f(
            imu_config["gyro_bias"][0].get<float>(),
            imu_config["gyro_bias"][1].get<float>(),
            imu_config["gyro_bias"][2].get<float>()
        );
        accelerometer_bias = Vector3f(
            imu_config["accel_bias"][0].get<float>(),
            imu_config["accel_bias"][1].get<float>(),
            imu_config["accel_bias"][2].get<float>()
        );
    } catch (...) {
        cerr << "Failed to parse IMU biases." << endl;
    }

    return true;
}
