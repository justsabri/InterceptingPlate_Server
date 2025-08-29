#include <variant>
#include <mutex>
#include <stdexcept>
#include <vector>
#include "motor.h"
class MotorController
{
private:
    std::mutex mtx_;
    float min_reverse_position;
    float max_forward_positon;
public:
      // 指令枚举类型
    enum class Command {
        POSITION_MODE_TARGET,   // 位置模式+目标位置
        MAX_FORWARD_ACC,        // 最大正向加速度
        MIN_REVERSE_ACC,        // 最小负向加速度
        MAX_FORWARD_SPEED,      // 最大正向速度
        MIN_REVERSE_SPEED,      // 最小负向速度
        MAX_FORWARD_POS,        // 最大正向位置
        MIN_REVERSE_POS,        // 最小负向位置
        POSITION_OFFSET,        // 位置偏移
        BAUD_RATE               // 波特率
    };
    //初始化函数，电机位置范围，查询电机是否在0位
    // void init_motor_position();
    // 统一控制函数
    void control_motor(Command cmd, int can_id, std::variant<double, int32_t> value);
};

