#include "motor_controller.h"
#include "log.h"
void MotorController::control_motor(Command cmd, int can_id, std::variant<double, int32_t> value){
    if (!allow_ctrl_) {
        return;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    // min_reverse_position = 100000;
    // max_forward_positon = min_reverse_position + 2202738;
    switch(cmd) {
        case Command::POSITION_MODE_TARGET:
            {
                auto ptr = std::get_if<double>(&value);
                //判断目标位置是否越位
                AINFO << "canid " << can_id << " " << *ptr;
                MotorParser::getInstance().setPositionModeAndTarget(*ptr, can_id);
                // if(*ptr > min_reverse_position || *ptr < max_forward_positon ){
                // MotorParser::getInstance().setPositionModeAndTarget(*ptr, can_id);}
                break;
            }

        case Command::MAX_FORWARD_ACC:
            {
                auto ptr = std::get_if<int32_t>(&value);
                MotorParser::getInstance().setMaxForwardAcceleration(*ptr, can_id);
                break;
            }

        case Command::MIN_REVERSE_ACC:
            {
                auto ptr = std::get_if<int32_t>(&value);
                MotorParser::getInstance().setMinReverseAcceleration(*ptr, can_id);
                break;
            }

        case Command::MAX_FORWARD_SPEED:
            {
                auto ptr = std::get_if<double>(&value);
                MotorParser::getInstance().setMaxForwardSpeed(*ptr, can_id);
                break;
            }

        case Command::MIN_REVERSE_SPEED:
            {
                auto ptr = std::get_if<double>(&value);
                MotorParser::getInstance().setMinReverseSpeed(*ptr, can_id);
                break;
            }

        case Command::MAX_FORWARD_POS:
            {
                auto ptr = std::get_if<double>(&value);
                MotorParser::getInstance().setMaxForwardPosition(*ptr, can_id);
                break;
            }

        case Command::MIN_REVERSE_POS:
            {
                auto ptr = std::get_if<double>(&value);
                MotorParser::getInstance().setMinReversePosition(*ptr, can_id);
                break;
            }

        case Command::POSITION_OFFSET:
            {
                auto ptr = std::get_if<int32_t>(&value);
                MotorParser::getInstance().setPositionOffset(*ptr, can_id);
                break;
            }

        case Command::BAUD_RATE:
            {
                auto ptr = std::get_if<int32_t>(&value);
                MotorParser::getInstance().setBaud(*ptr, can_id);
                break;
            }

        default:
            AERROR << "Unknown command type";
    }
}

void MotorController::stopMotor(int can_id) {
    allow_ctrl_ = false;
    MotorParser::getInstance().setStopMode(can_id);
}

void MotorController::keepRunning() {
    if (!allow_ctrl_) {
        allow_ctrl_ = true;
    }
}
