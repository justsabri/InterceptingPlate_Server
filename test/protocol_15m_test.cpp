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

}  // namespace

int main() {
    testControlCommand();
    testInterceptorStatus();
    testShipStatus();
    std::cout << "protocol_15m_test passed" << std::endl;
    return 0;
}