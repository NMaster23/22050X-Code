#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>

namespace serial_bridge {

constexpr uint16_t HOST_CMD_MAGIC = 0x5058;     // "PX" (22050X Command)
constexpr uint16_t ROBOT_TELEM_MAGIC = 0x544D;  // "TM" (Telemetry)

enum class ControllerId : uint8_t {
    LEMLIB_LATERAL   = 0,
    LEMLIB_ANGULAR   = 1,
    LEMLIB_BOOMERANG = 2,
    LEMLIB_SWING     = 3,
    CASCADE_LIFT     = 4,
};

enum class MotionCommandType : uint8_t {
    HEARTBEAT              = 0,
    MOVE_TO_POINT_FWD      = 1,
    MOVE_TO_POINT_REV      = 2,
    TURN_TO_HEADING        = 3,
    TURN_TO_POINT          = 4,
    MOVE_TO_POSE           = 5,
    SWING_TO_HEADING_LEFT  = 6,
    SWING_TO_HEADING_RIGHT = 7,
    SWING_TO_POINT_LEFT    = 8,
    SWING_TO_POINT_RIGHT   = 9,
    CASCADE_LIFT_STEP      = 10,
    RETURN_TO_ORIGIN       = 11,
    HOLD_POSE_DISTURBANCE  = 12,
    CHAINED_ENDURANCE_LAP  = 13,
    EMERGENCY_STOP         = 14,
};

namespace TriggerFlags {
    constexpr uint8_t EXECUTE_MOTION      = (1u << 0);
    constexpr uint8_t HOT_RELOAD_GAINS    = (1u << 1);
    constexpr uint8_t SYNC_VISION_POSE    = (1u << 2);
    constexpr uint8_t BOUNDARY_ABORT      = (1u << 3);
    constexpr uint8_t SIGN_FLIP_RESET     = (1u << 4);
    constexpr uint8_t RECONSTRUCT_CHASSIS = (1u << 5);
    constexpr uint8_t RESET_LIFT_ORIGIN   = (1u << 6);
}

enum class RobotMotionStatus : uint8_t {
    IDLE                 = 0,
    IN_MOTION            = 1,
    RETURNING_TO_ORIGIN  = 2,
    WATCHDOG_TRIPPED     = 3,
    STALL_PROTECT        = 4,
};

namespace WatchdogFlags {
    constexpr uint8_t NONE              = 0x00;
    constexpr uint8_t BOUNDARY_TRIP     = (1u << 0);
    constexpr uint8_t DRIVETRAIN_STALL  = (1u << 1);
    constexpr uint8_t LIFT_STALL        = (1u << 2);
    constexpr uint8_t HEARTBEAT_LOST    = (1u << 3);
    constexpr uint8_t THERMAL_WARNING   = (1u << 4);
}

/**
 * @brief Host-to-Robot Command Packet (104 bytes, 4-byte aligned, Pod compatible)
 */
struct alignas(4) HostCommandPacket {
    uint16_t magic;
    uint16_t seq;
    uint8_t controller_id;
    uint8_t motion_type;
    uint8_t trigger_flags;
    uint8_t reserved;

    float kp;
    float ki;
    float kd;
    float windup_range;
    float small_error;
    float small_error_timeout;
    float large_error;
    float large_error_timeout;
    float slew;
    float horizontal_drift;
    float lead;
    float max_speed;
    float min_speed;
    float early_exit_range;

    float target_x;
    float target_y;
    float target_theta;
    float target_lift_height;

    float vision_x;
    float vision_y;
    float vision_theta;
    float gravity_ff_mv;

    uint32_t timeout_ms;
    uint16_t crc16;
    uint16_t pad_tail;
};

static_assert(sizeof(HostCommandPacket) == 104, "HostCommandPacket must be exactly 104 bytes");

/**
 * @brief Robot-to-Host Telemetry Packet (76 bytes, 4-byte aligned, Pod compatible)
 */
struct alignas(4) RobotTelemetryPacket {
    uint16_t magic;
    uint16_t seq;
    uint32_t timestamp_ms;

    float pose_x;
    float pose_y;
    float pose_theta;
    float raw_imu_heading;

    float left_vel_rpm;
    float right_vel_rpm;
    float left_current_ma;
    float right_current_ma;
    float left_temp_c;
    float right_temp_c;

    float cascade_pos_in;
    float cascade_vel_rpm;
    float cascade_current_ma;
    float cascade_temp_c;

    uint32_t battery_voltage_mv;
    uint8_t motion_status;
    uint8_t active_controller_id;
    uint8_t watchdog_flags;
    uint8_t reserved;

    uint16_t crc16;
    uint16_t pad_tail;
};

static_assert(sizeof(RobotTelemetryPacket) == 76, "RobotTelemetryPacket must be exactly 76 bytes");

/**
 * @brief Compute CRC-16/CCITT-FALSE (poly = 0x1021, init = 0xFFFF)
 */
inline uint16_t compute_crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

/**
 * @brief Consistent Overhead Byte Stuffing (COBS) Encoder
 * @return Number of encoded bytes written to dst (not including trailing 0x00 delimiter)
 */
inline size_t cobs_encode(const uint8_t* src, size_t length, uint8_t* dst) {
    size_t read_index = 0;
    size_t write_index = 1;
    size_t code_index = 0;
    uint8_t code = 1;

    while (read_index < length) {
        if (src[read_index] == 0) {
            dst[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        } else {
            dst[write_index++] = src[read_index++];
            code++;
            if (code == 0xFF) {
                dst[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }
    dst[code_index] = code;
    return write_index;
}

/**
 * @brief Consistent Overhead Byte Stuffing (COBS) Decoder
 * @return Number of decoded bytes written to dst, or 0 on framing error
 */
inline size_t cobs_decode(const uint8_t* src, size_t length, uint8_t* dst) {
    if (length == 0) return 0;
    size_t read_index = 0;
    size_t write_index = 0;

    while (read_index < length) {
        uint8_t code = src[read_index];
        if (code == 0 || (read_index + code > length && code != 1)) {
            return 0;
        }
        read_index++;
        for (uint8_t i = 1; i < code; ++i) {
            dst[write_index++] = src[read_index++];
        }
        if (code != 0xFF && read_index < length) {
            dst[write_index++] = 0;
        }
    }
    return write_index;
}

inline bool validate_command_packet(const HostCommandPacket& pkt) {
    if (pkt.magic != HOST_CMD_MAGIC) return false;
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&pkt);
    uint16_t expected_crc = compute_crc16(raw, offsetof(HostCommandPacket, crc16));
    return pkt.crc16 == expected_crc;
}

inline void finalize_telemetry_packet(RobotTelemetryPacket& pkt) {
    pkt.magic = ROBOT_TELEM_MAGIC;
    pkt.pad_tail = 0;
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&pkt);
    pkt.crc16 = compute_crc16(raw, offsetof(RobotTelemetryPacket, crc16));
}

} // namespace serial_bridge
