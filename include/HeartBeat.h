#pragma once

#include <iostream>
#include <functional>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include <condition_variable>

class Heartbeat {
public:
    using Callback = std::function<void()>;

    enum class State {
        NORMAL,
        TIMEOUT
    };

    Heartbeat(int interval_ms = 1000, int timeout_ms = 5000)
        : interval(interval_ms),
          timeout(timeout_ms),
          running(false),
          state(State::NORMAL),
          lastBeat(std::chrono::steady_clock::now()) {}

    ~Heartbeat() {
        stop();
    }

    void setBeatCallback(Callback cb) {
        std::lock_guard<std::mutex> lock(mtx);
        beatCallback = cb;
    }

    void setTimeoutCallback(Callback cb) {
        std::lock_guard<std::mutex> lock(mtx);
        timeoutCallback = cb;
    }

    void setRecoverCallback(Callback cb) {
        std::lock_guard<std::mutex> lock(mtx);
        recoverCallback = cb;
    }

    // 模块调用：喂狗，续上心跳
    void feed() {
        std::lock_guard<std::mutex> lock(mtx);
        lastBeat = std::chrono::steady_clock::now();

        // 如果之前是超时状态，现在要恢复
        if (state == State::TIMEOUT) {
            state = State::NORMAL;
            if (recoverCallback) {
                recoverCallback();
            }
        }
    }

    void start() {
        if (running) return;
        running = true;
        worker = std::thread([this]() { this->run(); });
    }

    void stop() {
        running = false;
        cv.notify_all();
        if (worker.joinable()) {
            worker.join();
        }
    }

private:
    void run() {
        auto next = std::chrono::steady_clock::now();
        while (running) {
            next += std::chrono::milliseconds(interval);

            {
                std::lock_guard<std::mutex> lock(mtx);

                if (beatCallback) {
                    beatCallback();
                }

                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastBeat).count();

                if (elapsed > timeout && state == State::NORMAL) {
                    state = State::TIMEOUT;
                    if (timeoutCallback) {
                        timeoutCallback();
                    }
                }
            }

            std::unique_lock<std::mutex> lock(cv_mtx);
            cv.wait_until(lock, next, [this]() { return !running; });
        }
    }

private:
    int interval;
    int timeout;
    std::atomic<bool> running;
    std::thread worker;

    std::mutex mtx;
    std::condition_variable cv;
    std::mutex cv_mtx;

    Callback beatCallback;
    Callback timeoutCallback;
    Callback recoverCallback;

    State state;
    std::chrono::steady_clock::time_point lastBeat;
};

// ==========================
// 使用示例
// ==========================
// int main() {
//     Heartbeat hb(1000, 3000);

//     hb.setBeatCallback([]() {
//         std::cout << "[Heartbeat] tick..." << std::endl;
//     });

//     hb.setTimeoutCallback([]() {
//         std::cout << "[Heartbeat] TIMEOUT !!!" << std::endl;
//     });

//     hb.setRecoverCallback([]() {
//         std::cout << "[Heartbeat] RECOVERED ✅" << std::endl;
//     });

//     hb.start();

//     // 模拟：前5秒正常喂狗 → 然后停止喂狗5秒（超时）→ 再恢复喂狗
//     std::this_thread::sleep_for(std::chrono::seconds(5));
//     for (int i = 0; i < 5; i++) {
//         hb.feed();
//         std::cout << "[Feeder] feed()" << std::endl;
//         std::this_thread::sleep_for(std::chrono::seconds(1));
//     }

//     std::cout << "[Test] 停止喂狗..." << std::endl;
//     std::this_thread::sleep_for(std::chrono::seconds(5));

//     std::cout << "[Test] 再次喂狗..." << std::endl;
//     hb.feed();  // 应该触发 RECOVER

//     std::this_thread::sleep_for(std::chrono::seconds(3));
//     hb.stop();

//     return 0;
// }
