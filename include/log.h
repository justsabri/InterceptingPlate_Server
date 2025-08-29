/**
 * @file:    log.h
 * @brief:   谷歌日志glog封装:宏实现
 *
 *
 * @author:  Wesley
 * @date:    2025-07-04 17:21:25
 * @note:
 */

/**
使用示例
#include <glog/logging.h>
#include <log.h>
#include <iostream>

int main(int argc, char* argv[]) {
  // 日志初始化
  InitLog("MyApp");

  // 输出各种级别的日志
  ADEBUG << "这是一个ADEBUG信息.";
  AINFO << "这是一个AINFO信息.";
  AWARN << "这是一个AWARN信息.";
  AERROR << "这是一个AERROR信息.";

  // 日志结束
  StopLog();

  return 0;
}
*/

#ifndef LOG_H_
#define LOG_H_

#include <cstdarg>
#include <string>
#include <csignal>
#include "glog/logging.h"
#include "glog/raw_logging.h"
#include <execinfo.h>  // 用于获取堆栈信息
#include <filesystem>
#include <fstream>

static void HandleSignal(int signal) {
    void* callstack[128];
    int frames = backtrace(callstack, 128);
    char** strs = backtrace_symbols(callstack, frames);

    // 打印堆栈信息
    LOG(ERROR) << "Received signal " << signal << ", stack trace:";
    for (int i = 0; i < frames; i++) {
        LOG(ERROR) << strs[i];
    }
    free(strs);

    // 退出程序
    exit(1);
}

#define LEFT_BRACKET "["
#define RIGHT_BRACKET "]"

//-----------------------------日志初始化-----------------------------
// #ifndef MODULE_NAME
// #  define MODULE_NAME "MyApp"
// #  endif
// 静态变量定义
static std::string MODULE_NAME = "MyApp";
//-----------------------------日志初始化-----------------------------

#define ADEBUG_MODULE(module) \
  VLOG(0) << LEFT_BRACKET << module << RIGHT_BRACKET << "[DEBUG]"

//-----------------------------日志初始化-----------------------------
// 日志初始化
static void InitLog(const char* module_name = "MyApp",
                    const char* log_dir = "/home/zlg/server/0730/log") {
  namespace fs = std::filesystem;

  fs::path log_path = fs::current_path() / "log";
  // 初始化glog
  google::InitGoogleLogging(module_name);
  google::InstallFailureSignalHandler();  // 注册崩溃信号处理程序

  // 自定义信号处理
  signal(SIGSEGV, HandleSignal);  // 捕获段错误信号
  signal(SIGABRT, HandleSignal);  // 捕获 abort 信号
  
  MODULE_NAME = module_name;
  // 设置日志输出目录
  FLAGS_log_dir = log_path;
  // 设置日志文件名，格式如：program_name.YYYY-MM-DD_HH-MM-SS.XXX.log
  FLAGS_log_prefix = true;
  // 设置日志的最小级别
  FLAGS_minloglevel = google::INFO;  // 默认输出 INFO 及以上级别的日志
  // 同时输出到终端
  FLAGS_alsologtostderr = 1;  // 同时输出到终端
  // 设置终端颜色
  FLAGS_colorlogtostderr = true;  // Set log color
  // 设置单个日志文件的最大大小（以MB为单位）。当日志文件达到此大小时，glog将自动开始写入一个新的日志文件
  FLAGS_max_log_size = 1024;
  // 当磁盘空间不足时，是否停止日志记录。
  FLAGS_stop_logging_if_full_disk = true;  // If disk is full
  FLAGS_logbufsecs = 0;               // 实时刷新日志

  // 设置日志级别为 DEBUG  // 设置VLOG日志级别为4（DEBUG级别）
  google::SetVLOGLevel(module_name, 4);
  // 如下设置,可以确保只有INFO信息会输出到文件
  google::SetLogDestination(google::WARNING, "");
  google::SetLogDestination(google::ERROR, "");
  google::SetLogDestination(google::FATAL, "");

}
//------------------------------日志结束-----------------------------
// 日志结束
static void StopLog() { google::ShutdownGoogleLogging(); }

//-----------------------几种等级日志打印-----------------------------
#define ADEBUG ADEBUG_MODULE(MODULE_NAME)
#define AINFO ALOG_MODULE(MODULE_NAME, INFO)
#define AWARN ALOG_MODULE(MODULE_NAME, WARN)
#define AERROR ALOG_MODULE(MODULE_NAME, ERROR)
// AFATAL 会结束程序
#define AFATAL ALOG_MODULE(MODULE_NAME, FATAL)
/*
调用示例：
  ADEBUG << "这是一个ADEBUG信息.";
  AINFO << "这是一个AINFO信息.";
  AWARN << "这是一个AWARN信息.";
  AERROR << "这是一个AERROR信息.";
*/
//-----------------------几种等级日志打印-----------------------------

#ifndef ALOG_MODULE_STREAM
#  define ALOG_MODULE_STREAM(log_severity) ALOG_MODULE_STREAM_##log_severity
#endif

#ifndef ALOG_MODULE
#  define ALOG_MODULE(module, log_severity) \
    ALOG_MODULE_STREAM(log_severity)(module)
#endif

#define ALOG_MODULE_STREAM_INFO(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::INFO).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define ALOG_MODULE_STREAM_WARN(module)                            \
  google::LogMessage(__FILE__, __LINE__, google::WARNING).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define ALOG_MODULE_STREAM_ERROR(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::ERROR).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define ALOG_MODULE_STREAM_FATAL(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::FATAL).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define AINFO_IF(cond) ALOG_IF(INFO, cond, MODULE_NAME)
#define AWARN_IF(cond) ALOG_IF(WARN, cond, MODULE_NAME)
#define AERROR_IF(cond) ALOG_IF(ERROR, cond, MODULE_NAME)
#define AFATAL_IF(cond) ALOG_IF(FATAL, cond, MODULE_NAME)
#define ALOG_IF(severity, cond, module) \
  !(cond) ? (void)0                     \
          : google::LogMessageVoidify() & ALOG_MODULE(module, severity)

#define ACHECK(cond) CHECK(cond) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET

#define AINFO_EVERY(freq) \
  LOG_EVERY_N(INFO, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET
#define AWARN_EVERY(freq) \
  LOG_EVERY_N(WARNING, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET
#define AERROR_EVERY(freq) \
  LOG_EVERY_N(ERROR, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET

#if !defined(RETURN_IF_NULL)
#  define RETURN_IF_NULL(ptr)          \
    if (ptr == nullptr) {              \
      AWARN << #ptr << " is nullptr."; \
      return;                          \
    }
#endif

#if !defined(RETURN_VAL_IF_NULL)
#  define RETURN_VAL_IF_NULL(ptr, val) \
    if (ptr == nullptr) {              \
      AWARN << #ptr << " is nullptr."; \
      return val;                      \
    }
#endif

#if !defined(RETURN_IF)
#  define RETURN_IF(condition)           \
    if (condition) {                     \
      AWARN << #condition << " is met."; \
      return;                            \
    }
#endif

#if !defined(RETURN_VAL_IF)
#  define RETURN_VAL_IF(condition, val)  \
    if (condition) {                     \
      AWARN << #condition << " is met."; \
      return val;                        \
    }
#endif

#if !defined(_RETURN_VAL_IF_NULL2__)
#  define _RETURN_VAL_IF_NULL2__
#  define RETURN_VAL_IF_NULL2(ptr, val) \
    if (ptr == nullptr) {               \
      return (val);                     \
    }
#endif

#if !defined(_RETURN_VAL_IF2__)
#  define _RETURN_VAL_IF2__
#  define RETURN_VAL_IF2(condition, val) \
    if (condition) {                     \
      return (val);                      \
    }
#endif

#if !defined(_RETURN_IF2__)
#  define _RETURN_IF2__
#  define RETURN_IF2(condition) \
    if (condition) {            \
      return;                   \
    }
#endif

#endif  // LOG_H_
