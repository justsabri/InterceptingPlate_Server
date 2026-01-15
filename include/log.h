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
#include <vector>
#include <algorithm>
#include <chrono>
#include <thread>
#include <atomic>
#include "glog/logging.h"
#include "glog/raw_logging.h"
#include <execinfo.h> // 用于获取堆栈信息
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

// 静态变量定义
static std::string MODULE_NAME = "MyApp";
static std::atomic<bool> log_cleaner_running(false);
static std::thread log_cleaner_thread;
static int MAX_LOG_FILES = 10; // 默认最大日志文件数量

static void HandleSignal(int signal)
{
  void *callstack[128];
  int frames = backtrace(callstack, 128);
  char **strs = backtrace_symbols(callstack, frames);

  // 打印堆栈信息
  LOG(ERROR) << "Received signal " << signal << ", stack trace:";
  for (int i = 0; i < frames; i++)
  {
    LOG(ERROR) << strs[i];
  }
  free(strs);

  // 退出程序
  exit(1);
}

#define LEFT_BRACKET "["
#define RIGHT_BRACKET "]"

//----------------------------- 日志清理函数 -----------------------------
static void CleanOldLogs(const std::string &module_name) {
    try {
        if (FLAGS_log_dir.empty()) {
            LOG(WARNING) << "Log directory not set, skip cleaning";
            return;
        }

        LOG(INFO) << "Starting log cleanup for: " << module_name;
        
        std::vector<fs::path> log_files;
        int total_files = 0;
        
        for (const auto &entry : fs::directory_iterator(FLAGS_log_dir)) {
            total_files++;
            std::string filename = entry.path().filename().string();
            
            if (entry.is_regular_file() && 
                filename.find(module_name) != std::string::npos) {
                log_files.push_back(entry.path());
                LOG(INFO) << "Found log file: " << filename;
            }
        }

        LOG(INFO) << "Total files: " << total_files 
                  << ", Log files: " << log_files.size()
                  << ", Max allowed: " << MAX_LOG_FILES;

        if (log_files.size() <= MAX_LOG_FILES) {
            LOG(INFO) << "No cleanup needed";
            return;
        }

        // 按修改时间排序（旧文件在前）
        std::sort(log_files.begin(), log_files.end(),
                  [](const fs::path &a, const fs::path &b) {
                      return fs::last_write_time(a) < fs::last_write_time(b);
                  });

        int files_to_delete = log_files.size() - MAX_LOG_FILES;
        LOG(INFO) << "Deleting " << files_to_delete << " old log files";
        
        for (int i = 0; i < files_to_delete; ++i) {
            try {
                std::string path_str = log_files[i].string();
                if (fs::remove(log_files[i])) {
                    LOG(INFO) << "Deleted: " << path_str;
                } else {
                    LOG(ERROR) << "Failed to delete: " << path_str;
                }
            } catch (const fs::filesystem_error &e) {
                LOG(ERROR) << "Error deleting file: " << e.what();
            }
        }
    } catch (const fs::filesystem_error &e) {
        LOG(ERROR) << "Filesystem error: " << e.what();
    } catch (const std::exception &e) {
        LOG(ERROR) << "Unexpected error: " << e.what();
    }
}

//----------------------------- 日志清理线程 -----------------------------
static void LogCleanerTask()
{
  while (log_cleaner_running)
  {
    std::this_thread::sleep_for(std::chrono::hours(1)); // 每小时清理一次
    CleanOldLogs(MODULE_NAME);
  }
}

//-----------------------------日志初始化-----------------------------
// #ifndef MODULE_NAME
// #  define MODULE_NAME "MyApp"
// #  endif

//-----------------------------日志初始化-----------------------------

//-----------------------------日志初始化-----------------------------
// 日志初始化
static void InitLog(const char *module_name = "MyApp",
                    const char *log_dir = "/home/zlg/server/InterceptingPlate_Server/log",
                    int max_log_files = 5)
{
  namespace fs = std::filesystem;

  fs::path log_path = log_dir;
  MAX_LOG_FILES = max_log_files;

  // 确保日志目录存在
  if (!fs::exists(log_path))
  {
    fs::create_directories(log_path);
  }

  // 初始化glog
  google::InitGoogleLogging(module_name);
  google::InstallFailureSignalHandler(); // 注册崩溃信号处理程序

  // 自定义信号处理
  signal(SIGSEGV, HandleSignal); // 捕获段错误信号
  signal(SIGABRT, HandleSignal); // 捕获 abort 信号

  MODULE_NAME = module_name;
  // 设置日志输出目录
  FLAGS_log_dir = log_path.string();
  // 设置日志文件名，格式如：program_name.YYYY-MM-DD_HH-MM-SS.XXX.log
  FLAGS_log_prefix = true;
  // 设置日志的最小级别
  FLAGS_minloglevel = google::INFO; // 默认输出 INFO 及以上级别的日志
  // 同时输出到终端
  FLAGS_alsologtostderr = 1; // 同时输出到终端
  if (module_name == "Test")
  {
    FLAGS_alsologtostderr = 0;
  }
  // 设置终端颜色
  FLAGS_colorlogtostderr = true; // Set log color
  // 设置单个日志文件的最大大小（以MB为单位）。当日志文件达到此大小时，glog将自动开始写入一个新的日志文件
  FLAGS_max_log_size = 256;

  // 当磁盘空间不足时，是否停止日志记录。
  FLAGS_stop_logging_if_full_disk = true; // If disk is full
  FLAGS_logbufsecs = 0;                   // 实时刷新日志

  // 设置日志级别为 DEBUG  // 设置VLOG日志级别为4（DEBUG级别）
  google::SetVLOGLevel(module_name, 4);
  // 如下设置,可以确保只有INFO信息会输出到文件
  google::SetLogDestination(google::WARNING, "");
  google::SetLogDestination(google::ERROR, "");
  google::SetLogDestination(google::FATAL, "");

  // 清理旧日志并启动清理线程
  CleanOldLogs(module_name);
  log_cleaner_running = true;
  log_cleaner_thread = std::thread(LogCleanerTask);
}
//------------------------------日志结束-----------------------------
// 日志结束
static void StopLog()
{
  log_cleaner_running = false;
  if (log_cleaner_thread.joinable())
  {
    log_cleaner_thread.join();
  }
  google::ShutdownGoogleLogging();
}

//-----------------------几种等级日志打印-----------------------------
#define ADEBUG_MODULE(module) \
  VLOG(0) << LEFT_BRACKET << module << RIGHT_BRACKET << "[DEBUG]"
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
#define ALOG_MODULE_STREAM(log_severity) ALOG_MODULE_STREAM_##log_severity
#endif

#ifndef ALOG_MODULE
#define ALOG_MODULE(module, log_severity) \
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
#define RETURN_IF_NULL(ptr)          \
  if (ptr == nullptr)                \
  {                                  \
    AWARN << #ptr << " is nullptr."; \
    return;                          \
  }
#endif

#if !defined(RETURN_VAL_IF_NULL)
#define RETURN_VAL_IF_NULL(ptr, val) \
  if (ptr == nullptr)                \
  {                                  \
    AWARN << #ptr << " is nullptr."; \
    return val;                      \
  }
#endif

#if !defined(RETURN_IF)
#define RETURN_IF(condition)           \
  if (condition)                       \
  {                                    \
    AWARN << #condition << " is met."; \
    return;                            \
  }
#endif

#if !defined(RETURN_VAL_IF)
#define RETURN_VAL_IF(condition, val)  \
  if (condition)                       \
  {                                    \
    AWARN << #condition << " is met."; \
    return val;                        \
  }
#endif

#if !defined(_RETURN_VAL_IF_NULL2__)
#define _RETURN_VAL_IF_NULL2__
#define RETURN_VAL_IF_NULL2(ptr, val) \
  if (ptr == nullptr)                 \
  {                                   \
    return (val);                     \
  }
#endif

#if !defined(_RETURN_VAL_IF2__)
#define _RETURN_VAL_IF2__
#define RETURN_VAL_IF2(condition, val) \
  if (condition)                       \
  {                                    \
    return (val);                      \
  }
#endif

#if !defined(_RETURN_IF2__)
#define _RETURN_IF2__
#define RETURN_IF2(condition) \
  if (condition)              \
  {                           \
    return;                   \
  }
#endif

#endif // LOG_H_
