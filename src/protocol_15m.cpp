#include "protocol_15m.h"

#include <cstring>
#include <limits>

namespace protocol_15m {
namespace {

bool isRatio(float value) {
    return value >= 0.0f && value <= 1.0f;
}

bool isControlMode(uint16_t value) {
    return value == 1 || value == 2;
}

}  // namespace

DecodeResult DecodeResult::success() {
    return {true, ""};
}

DecodeResult DecodeResult::failure(const std::string& message) {
    return {false, message};
}

uint16_t readUint16BE(const uint8_t* data) {
    return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) |
                                 static_cast<uint16_t>(data[1]));
}

int16_t readInt16BE(const uint8_t* data) {
    return static_cast<int16_t>(readUint16BE(data));
}

uint64_t readUint64BE(const uint8_t* data) {
    uint64_t value = 0;
    for (size_t i = 0; i < 8; ++i) {
        value = (value << 8) | static_cast<uint64_t>(data[i]);
    }
    return value;
}

float readFloatBE(const uint8_t* data) {
    uint32_t raw = (static_cast<uint32_t>(data[0]) << 24) |
                   (static_cast<uint32_t>(data[1]) << 16) |
                   (static_cast<uint32_t>(data[2]) << 8) |
                   static_cast<uint32_t>(data[3]);
    float value = 0.0f;
    std::memcpy(&value, &raw, sizeof(value));
    return value;
}

double readDoubleBE(const uint8_t* data) {
    uint64_t raw = readUint64BE(data);
    double value = 0.0;
    std::memcpy(&value, &raw, sizeof(value));
    return value;
}

void writeUint16BE(uint8_t* data, uint16_t value) {
    data[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data[1] = static_cast<uint8_t>(value & 0xFF);
}

void writeInt16BE(uint8_t* data, int16_t value) {
    writeUint16BE(data, static_cast<uint16_t>(value));
}

void writeUint64BE(uint8_t* data, uint64_t value) {
    for (size_t i = 0; i < 8; ++i) {
        data[7 - i] = static_cast<uint8_t>(value & 0xFF);
        value >>= 8;
    }
}

void writeFloatBE(uint8_t* data, float value) {
    uint32_t raw = 0;
    std::memcpy(&raw, &value, sizeof(raw));
    data[0] = static_cast<uint8_t>((raw >> 24) & 0xFF);
    data[1] = static_cast<uint8_t>((raw >> 16) & 0xFF);
    data[2] = static_cast<uint8_t>((raw >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>(raw & 0xFF);
}

void writeDoubleBE(uint8_t* data, double value) {
    uint64_t raw = 0;
    std::memcpy(&raw, &value, sizeof(raw));
    writeUint64BE(data, raw);
}

DecodeResult decodeHeader(const uint8_t* data, size_t length, FrameHeader& out) {
    if (data == nullptr) {
        return DecodeResult::failure("null frame buffer");
    }
    if (length < kHeaderLength) {
        return DecodeResult::failure("frame shorter than header");
    }

    out.msg_type = readUint16BE(data);
    out.payload_length = readUint16BE(data + 2);
    if (length < kHeaderLength + out.payload_length) {
        return DecodeResult::failure("incomplete frame");
    }
    return DecodeResult::success();
}

DecodeResult decodeControlCommand(const uint8_t* data, size_t length, ControlCommand& out) {
    FrameHeader header;
    DecodeResult header_result = decodeHeader(data, length, header);
    if (!header_result.ok) {
        return header_result;
    }
    if (header.msg_type != kMsgTypeControlCommand) {
        return DecodeResult::failure("unexpected control command message type");
    }
    if (header.payload_length != kControlCommandPayloadLength) {
        return DecodeResult::failure("invalid control command payload length");
    }

    size_t offset = kHeaderLength;
    out.control_mode = readUint16BE(data + offset);
    offset += 2;
    out.auto_mode_param = readUint16BE(data + offset);
    offset += 2;
    out.manual_left_extend = readFloatBE(data + offset);
    offset += 4;
    out.manual_right_extend = readFloatBE(data + offset);
    offset += 4;
    out.timestamp_ms = readUint64BE(data + offset);

    if (!isControlMode(out.control_mode)) {
        return DecodeResult::failure("invalid control mode");
    }
    if (!isRatio(out.manual_left_extend) || !isRatio(out.manual_right_extend)) {
        return DecodeResult::failure("manual extension ratio out of range");
    }

    return DecodeResult::success();
}

DecodeResult decodeShipStatus(const uint8_t* data, size_t length, ShipStatus& out) {
    FrameHeader header;
    DecodeResult header_result = decodeHeader(data, length, header);
    if (!header_result.ok) {
        return header_result;
    }
    if (header.msg_type != kMsgTypeShipStatus) {
        return DecodeResult::failure("unexpected ship status message type");
    }
    if (header.payload_length != kShipStatusPayloadLength) {
        return DecodeResult::failure("invalid ship status payload length");
    }

    size_t offset = kHeaderLength;
    out.roll = readFloatBE(data + offset);
    offset += 4;
    out.pitch = readFloatBE(data + offset);
    offset += 4;
    out.rudder = readFloatBE(data + offset);
    offset += 4;
    out.speed = readFloatBE(data + offset);
    offset += 4;
    out.longitude = readDoubleBE(data + offset);
    offset += 8;
    out.latitude = readDoubleBE(data + offset);
    offset += 8;
    out.left_engine_speed = readFloatBE(data + offset);
    offset += 4;
    out.right_engine_speed = readFloatBE(data + offset);
    offset += 4;
    out.left_engine_gear = readInt16BE(data + offset);
    offset += 2;
    out.right_engine_gear = readInt16BE(data + offset);
    offset += 2;
    out.timestamp = readDoubleBE(data + offset);

    return DecodeResult::success();
}

size_t encodeInterceptorStatus(const InterceptorStatus& status, uint8_t* out, size_t capacity) {
    if (out == nullptr || capacity < kInterceptorStatusFrameLength) {
        return 0;
    }

    size_t offset = 0;
    writeUint16BE(out + offset, kMsgTypeInterceptorStatus);
    offset += 2;
    writeUint16BE(out + offset, kInterceptorStatusPayloadLength);
    offset += 2;
    writeFloatBE(out + offset, status.current_speed);
    offset += 4;
    writeFloatBE(out + offset, status.left_extend_threshold);
    offset += 4;
    writeFloatBE(out + offset, status.right_extend_threshold);
    offset += 4;
    writeFloatBE(out + offset, status.left_current_extend);
    offset += 4;
    writeFloatBE(out + offset, status.right_current_extend);
    offset += 4;
    writeUint16BE(out + offset, status.motor_count);
    offset += 2;
    writeUint16BE(out + offset, status.motor1_status);
    offset += 2;
    writeUint16BE(out + offset, status.motor2_status);
    offset += 2;
    writeUint16BE(out + offset, status.motor3_status);
    offset += 2;
    writeUint16BE(out + offset, status.motor4_status);
    offset += 2;
    writeUint16BE(out + offset, status.imu_status);
    offset += 2;
    writeUint16BE(out + offset, status.slave_status);
    offset += 2;
    writeFloatBE(out + offset, status.current_pitch);
    offset += 4;
    writeFloatBE(out + offset, status.current_roll);
    offset += 4;

    return offset;
}

}  // namespace protocol_15m
