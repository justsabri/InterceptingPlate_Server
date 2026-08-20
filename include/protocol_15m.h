#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace protocol_15m {

constexpr uint16_t kMsgTypeControlCommand = 0;
constexpr uint16_t kMsgTypeInterceptorStatus = 1;
constexpr uint16_t kMsgTypeShipStatus = 2;

constexpr size_t kHeaderLength = 4;
constexpr size_t kControlCommandPayloadLength = 20;
constexpr size_t kInterceptorStatusPayloadLength = 42;
constexpr size_t kShipStatusPayloadLength = 52;

constexpr size_t kControlCommandFrameLength = kHeaderLength + kControlCommandPayloadLength;
constexpr size_t kInterceptorStatusFrameLength = kHeaderLength + kInterceptorStatusPayloadLength;
constexpr size_t kShipStatusFrameLength = kHeaderLength + kShipStatusPayloadLength;

struct ControlCommand {
    uint16_t control_mode = 0;
    uint16_t auto_mode_param = 0;
    float manual_left_extend = 0.0f;
    float manual_right_extend = 0.0f;
    uint64_t timestamp_ms = 0;
};

struct InterceptorStatus {
    float current_speed = 0.0f;
    float left_extend_threshold = 0.0f;
    float right_extend_threshold = 0.0f;
    float left_current_extend = 0.0f;
    float right_current_extend = 0.0f;
    uint16_t motor_count = 0;
    uint16_t motor1_status = 101;
    uint16_t motor2_status = 101;
    uint16_t motor3_status = 101;
    uint16_t motor4_status = 101;
    uint16_t imu_status = 201;
    uint16_t slave_status = 301;
    float current_pitch = 0.0f;
    float current_roll = 0.0f;
};

struct ShipStatus {
    float roll = 0.0f;
    float pitch = 0.0f;
    float rudder = 0.0f;
    float speed = 0.0f;
    double longitude = 0.0;
    double latitude = 0.0;
    float left_engine_speed = 0.0f;
    float right_engine_speed = 0.0f;
    int16_t left_engine_gear = 0;
    int16_t right_engine_gear = 0;
    double timestamp = 0.0;
};

struct DecodeResult {
    bool ok = false;
    std::string error;

    static DecodeResult success();
    static DecodeResult failure(const std::string& message);
};

struct FrameHeader {
    uint16_t msg_type = 0;
    uint16_t payload_length = 0;
};

uint16_t readUint16BE(const uint8_t* data);
int16_t readInt16BE(const uint8_t* data);
uint64_t readUint64BE(const uint8_t* data);
float readFloatBE(const uint8_t* data);
double readDoubleBE(const uint8_t* data);

void writeUint16BE(uint8_t* data, uint16_t value);
void writeInt16BE(uint8_t* data, int16_t value);
void writeUint64BE(uint8_t* data, uint64_t value);
void writeFloatBE(uint8_t* data, float value);
void writeDoubleBE(uint8_t* data, double value);

DecodeResult decodeHeader(const uint8_t* data, size_t length, FrameHeader& out);
DecodeResult decodeControlCommand(const uint8_t* data, size_t length, ControlCommand& out);
DecodeResult decodeShipStatus(const uint8_t* data, size_t length, ShipStatus& out);
DecodeResult extractFrame(std::vector<uint8_t>& stream_buffer, std::vector<uint8_t>& frame);

size_t encodeInterceptorStatus(const InterceptorStatus& status, uint8_t* out, size_t capacity);

}  // namespace protocol_15m
