/**
 * @file:    alg_processor.h
 * @brief:   PID算法调度数据处理：包含数据输入、数据处理、数据输出
 *
 *
 * @author:  Wesley
 * @date:    2025-07-21 10:09:25
 * @note:
 */

#ifndef ALG_PROCESSOR_H
#define ALG_PROCESSOR_H

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>

#include "pid_control.h"

#define AlgInput PID_Input
#define AlgResult PID_Output

class AlgProcessor {
 public:
  // 定义回调函数类型
  using Callback = std::function<void(AlgResult&)>;

  // 构造函数：初始化 stopFlag，确保对象处于有效状态
  AlgProcessor();

  // 析构函数：确保资源的释放
  ~AlgProcessor();

  // 设置回调函数
  void set_callback(Callback callback);

  // 添加数据到输入队列
  void add_data(const AlgInput& data);

  // 数据处理函数
  void process_data();

  // 停止处理
  void stop();

  // 清除数据
  void clear();

 private:
  // 输入数据队列，存储结构体
  std::queue<AlgInput> input_queue_;

  // 用于同步的互斥量
  std::mutex mtx_;
  // 用于控制输入队列的条件变量
  std::condition_variable cv_input_;

  // 停止标志
  std::atomic<bool> stop_flag_{false};

  // 回调函数
  Callback callback_ = nullptr;
};

#endif  // ALG_PROCESSOR_H
