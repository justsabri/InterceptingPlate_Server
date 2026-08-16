#include "protocol_15m.h"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

namespace {

bool nearlyEqual(float lhs, float rhs) {
    return std::fabs(lhs - rhs) < 0.0001f;
}

bool nearlyEqual(double lhs, double rhs) {
    return std::fabs(lhs - rhs) < 0.0000001;
}

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << std::endl;
        std::exit(1);
    }
}

void testControlCommand() {
    std::vector<uint8_t> frame(protocol_15m::kControlCommandFrameLength);
    protocol_15m::writeUint16BE(frame.data(), protocol_15m::kMsgTypeControlCommand);
    protocol_15m::writeUint16BE(frame.data() + 2, protocol_15m::kControlCommandPayloadLength);
    protocol_15m::writeUint16BE(frame.data() + 4, 1);
    protocol_15m::writeUint16BE(frame.data() + 6, 31);
    protocol_15m::writeFloatBE(frame.data() + 8, 0.25f);
    protocol_15m::writeFloatBE(frame.data() + 12, 0.75f);
    protocol_15m::writeUint64BE(frame.data() + 16, 123456789ULL);

    protocol_15m::ControlCommand command{};
    protocol_15m::DecodeResult result =
        protocol_15m::decodeControlCommand(frame.data(), frame.size(), command);

    require(result.ok, "control command should decode");
    require(command.control_mode == 1, "control mode");
    require(command.auto_mode_param == 31, "auto mode param");
    require(nearlyEqual(command.manual_left_extend, 0.25f), "manual left");
    require(nearlyEqual(command.manual_right_extend, 0.75f), "manual right");
    require(command.timestamp_ms == 123456789ULL, "timestamp");

    frame[5] = 0;
    result = protocol_15m::decodeControlCommand(frame.data(), frame.size(), command);
    require(!result.ok, "control mode 0 should be rejected");
}

void testInterceptorStatus() {
    protocol_15m::InterceptorStatus status{};
    status.current_speed = 12.5f;
    status.left_extend_threshold = 0.8f;
    status.right_extend_threshold = 0.7f;
    status.left_current_extend = 0.2f;
    status.right_current_extend = 0.3f;
    status.motor_count = 2;
    status.motor1_status = 101;
    status.motor2_status = 106;
    status.motor3_status = 101;
    status.motor4_status = 101;
    status.imu_status = 201;
    status.slave_status = 301;
    status.current_pitch = -1.5f;
    status.current_roll = 2.5f;

    std::vector<uint8_t> frame(protocol_15m::kInterceptorStatusFrameLength);
    size_t length = protocol_15m::encodeInterceptorStatus(status, frame.data(), frame.size());

    require(length == protocol_15m::kInterceptorStatusFrameLength, "interceptor frame length");
    require(protocol_15m::readUint16BE(frame.data()) == protocol_15m::kMsgTypeInterceptorStatus, "status msg type");
    require(protocol_15m::readUint16BE(frame.data() + 2) == protocol_15m::kInterceptorStatusPayloadLength,
            "status payload length excludes header");
    require(nearlyEqual(protocol_15m::readFloatBE(frame.data() + 4), 12.5f), "status speed");
    require(protocol_15m::readUint16BE(frame.data() + 24) == 2, "motor count");
    require(protocol_15m::readUint16BE(frame.data() + 30) == 101, "motor3 default status");
    require(nearlyEqual(protocol_15m::readFloatBE(frame.data() + 38), -1.5f), "pitch");
    require(nearlyEqual(protocol_15m::readFloatBE(frame.data() + 42), 2.5f), "roll");
}

void testShipStatus() {
    std::vector<uint8_t> frame(protocol_15m::kShipStatusFrameLength);
    protocol_15m::writeUint16BE(frame.data(), protocol_15m::kMsgTypeShipStatus);
    protocol_15m::writeUint16BE(frame.data() + 2, protocol_15m::kShipStatusPayloadLength);
    protocol_15m::writeFloatBE(frame.data() + 4, 1.25f);
    protocol_15m::writeFloatBE(frame.data() + 8, -2.5f);
    protocol_15m::writeFloatBE(frame.data() + 12, 3.75f);
    protocol_15m::writeFloatBE(frame.data() + 16, 22.0f);
    protocol_15m::writeDoubleBE(frame.data() + 20, 121.1234567);
    protocol_15m::writeDoubleBE(frame.data() + 28, 31.7654321);
    protocol_15m::writeFloatBE(frame.data() + 36, 1200.0f);
    protocol_15m::writeFloatBE(frame.data() + 40, 1300.0f);
    protocol_15m::writeInt16BE(frame.data() + 44, 1);
    protocol_15m::writeInt16BE(frame.data() + 46, -1);
    protocol_15m::writeDoubleBE(frame.data() + 48, 1786680000.5);

    protocol_15m::ShipStatus status{};
    protocol_15m::DecodeResult result =
        protocol_15m::decodeShipStatus(frame.data(), frame.size(), status);

    require(result.ok, "ship status should decode");
    require(nearlyEqual(status.roll, 1.25f), "ship roll");
    require(nearlyEqual(status.pitch, -2.5f), "ship pitch");
    require(nearlyEqual(status.rudder, 3.75f), "ship rudder");
    require(nearlyEqual(status.speed, 22.0f), "ship speed");
    require(nearlyEqual(status.longitude, 121.1234567), "ship longitude");
    require(nearlyEqual(status.latitude, 31.7654321), "ship latitude");
    require(status.left_engine_gear == 1, "left gear");
    require(status.right_engine_gear == -1, "right gear");
    require(nearlyEqual(status.timestamp, 1786680000.5), "ship timestamp");

    frame[3] = 0;
    result = protocol_15m::decodeShipStatus(frame.data(), frame.size(), status);
    require(!result.ok, "invalid ship payload length should fail");
}

void testFrameStreamExtraction() {
    std::vector<uint8_t> control_frame(protocol_15m::kControlCommandFrameLength);
    protocol_15m::writeUint16BE(control_frame.data(), protocol_15m::kMsgTypeControlCommand);
    protocol_15m::writeUint16BE(control_frame.data() + 2, protocol_15m::kControlCommandPayloadLength);
    protocol_15m::writeUint16BE(control_frame.data() + 4, 1);
    protocol_15m::writeUint16BE(control_frame.data() + 6, 35);
    protocol_15m::writeFloatBE(control_frame.data() + 8, 0.1f);
    protocol_15m::writeFloatBE(control_frame.data() + 12, 0.9f);
    protocol_15m::writeUint64BE(control_frame.data() + 16, 1111ULL);

    std::vector<uint8_t> ship_frame(protocol_15m::kShipStatusFrameLength);
    protocol_15m::writeUint16BE(ship_frame.data(), protocol_15m::kMsgTypeShipStatus);
    protocol_15m::writeUint16BE(ship_frame.data() + 2, protocol_15m::kShipStatusPayloadLength);
    protocol_15m::writeFloatBE(ship_frame.data() + 4, 1.0f);
    protocol_15m::writeFloatBE(ship_frame.data() + 8, 2.0f);
    protocol_15m::writeFloatBE(ship_frame.data() + 12, 3.0f);
    protocol_15m::writeFloatBE(ship_frame.data() + 16, 4.0f);
    protocol_15m::writeDoubleBE(ship_frame.data() + 20, 120.0);
    protocol_15m::writeDoubleBE(ship_frame.data() + 28, 30.0);
    protocol_15m::writeFloatBE(ship_frame.data() + 36, 500.0f);
    protocol_15m::writeFloatBE(ship_frame.data() + 40, 600.0f);
    protocol_15m::writeInt16BE(ship_frame.data() + 44, 1);
    protocol_15m::writeInt16BE(ship_frame.data() + 46, 2);
    protocol_15m::writeDoubleBE(ship_frame.data() + 48, 2222.5);

    std::vector<uint8_t> stream;
    stream.insert(stream.end(), control_frame.begin(), control_frame.begin() + 10);
    std::vector<uint8_t> extracted;
    protocol_15m::DecodeResult result = protocol_15m::extractFrame(stream, extracted);
    require(!result.ok, "partial frame should stay buffered");
    require(stream.size() == 10, "partial frame should not be consumed");

    stream.insert(stream.end(), control_frame.begin() + 10, control_frame.end());
    stream.insert(stream.end(), ship_frame.begin(), ship_frame.end());

    result = protocol_15m::extractFrame(stream, extracted);
    require(result.ok, "first full frame should extract");
    require(extracted.size() == protocol_15m::kControlCommandFrameLength, "first extracted size");
    protocol_15m::ControlCommand command{};
    result = protocol_15m::decodeControlCommand(extracted.data(), extracted.size(), command);
    require(result.ok, "extracted control should decode");
    require(command.auto_mode_param == 35, "extracted control mode");

    result = protocol_15m::extractFrame(stream, extracted);
    require(result.ok, "second full frame should extract");
    require(extracted.size() == protocol_15m::kShipStatusFrameLength, "second extracted size");
    protocol_15m::ShipStatus status{};
    result = protocol_15m::decodeShipStatus(extracted.data(), extracted.size(), status);
    require(result.ok, "extracted ship should decode");
    require(nearlyEqual(status.speed, 4.0f), "extracted ship speed");
    require(stream.empty(), "stream buffer should be empty after extraction");
}

}  // namespace

int main() {
    testControlCommand();
    testInterceptorStatus();
    testShipStatus();
    testFrameStreamExtraction();
    std::cout << "protocol_15m_test passed" << std::endl;
    return 0;
}
