#include <thread>
#include <atomic>
#include <unordered_map>
#include <nlohmann/json.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <arpa/inet.h>
#include <iostream>
#include <fstream>
#include <random>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <queue>
#include <fcntl.h>
#include <termios.h>
#include <sys/stat.h>  // 获取文件信息
// #include "log.h"

using json = nlohmann::json;

struct ProtocolItem {
    std::string describe;
    std::string type; // 数据单位转换
    int32_t value; 
};

class JsonHelper {
public:
    JsonHelper() :  lastModifyTime(0) {}
    // 获取文件最后修改时间
    time_t getFileModifyTime(const std::string& filename) {
        struct stat fileStat;
        std::unique_lock<std::mutex> l(file_mutex_);
        if (stat(filename.c_str(), &fileStat) == 0) {
            return fileStat.st_mtime;
        } else {
            return 0; // 文件不存在或无法访问
        }
    }

    // 安全读取 JSON
    bool safeReadJson(const std::string& filename, json& j) {
        try {
            std::unique_lock<std::mutex> l(file_mutex_);
            std::ifstream ifs(filename);
            if (!ifs.is_open()) {
                std::cerr << "Cannot open file: " << filename << std::endl;
                return false;
            }

            json temp;
            ifs >> temp;  // 尝试解析 JSON
            j = std::move(temp);
            return true;
        } catch (const json::parse_error& e) {
            std::cerr << "JSON parse error: " << e.what() << std::endl;
            return false;
        } catch (const std::exception& e) {
            std::cerr << "Other exception: " << e.what() << std::endl;
            return false;
        }
    }

    bool isNeedRead() {
        bool is_need = false;
        time_t curModifyTime = getFileModifyTime(jsonPath_);
        // std::cout << " curModifyTime " << jsonPath_ << ", " << curModifyTime << std::endl;
        is_need = curModifyTime != 0 && curModifyTime != lastModifyTime;
        if (is_need) {
            lastModifyTime = curModifyTime;
        }
        return is_need;
    }

public:
    std::string jsonPath_;
private:
    time_t lastModifyTime;
    std::mutex file_mutex_;
};

class VirtualMotor : public JsonHelper {
public:
    VirtualMotor(const std::string& ifname, const std::string& jsonPath)
        : ifname_(ifname), running_(false) {
            jsonPath_ = jsonPath;
            bool res = init();
            if (!res) {
                std::cerr << "motor init fail" << std::endl;
            }
        }

    ~VirtualMotor() {
        std::cout << "motor descont" << std::endl;
        stop();
        if (sock_ > 0) {
            close(sock_);
        }
    }

    bool init() {
        if (!loadConfig(jsonPath_)) {
            std::cerr << "Failed to load config." << std::endl;
            return false;
        }

        sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_ < 0) {
            perror("socket");
            return false;
        }

        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, ifname_.c_str(), IFNAMSIZ);
        if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
            perror("ioctl");
            return false;
        }

        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("bind");
            return false;
        }
        std::cout << "motor init this " << this << std::endl;
        return true;
    }

    void start() {
        std::cout << "motor this " << this << std::endl;
        running_ = true;
        tRecv_ = std::thread(&VirtualMotor::recvThread, this);
        tProc_ = std::thread(&VirtualMotor::processThread, this);
        tLoad_ = std::thread(&VirtualMotor::loadThread, this);
    }

    void stop() {
        running_ = false;
        cv_.notify_all();

        // 发送一帧can唤醒read
        system("cansend can0 123#11223344 &");

        if (tRecv_.joinable()) {
            tRecv_.join();
        }

        if (tProc_.joinable()) {
            tProc_.join();
        }

        if (tLoad_.joinable()) {
            tLoad_.join();
        }
    }

private:
    bool loadConfig(const std::string& path) {
        safeReadJson(path, config_);

        if (config_.contains("id")) {
            motorId_ = std::stoi(config_["id"].get<std::string>(), nullptr, 16);
        }

        if (config_.contains("protocol")) {
            std::unique_lock<std::mutex> l(protocol_mtx_);
            for (auto& [cmd, item] : config_["protocol"].items()) {
                ProtocolItem pi;
                pi.describe = item.value("discribe", "");
                pi.type     = item.value("type", "fixed");
                pi.value    = item.value("value", 0);
                std::cout << pi.describe << " value " << pi.value << std::endl;
                if (pi.type == "degree") {
                    pi.value = (pi.value / 360.0) * 121 * 65536.0;
                    std::cout << pi.describe << " convert value " << pi.value << std::endl;
                } else if (pi.type == "degree_per_s") {
                    pi.value = (pi.value * 121 * 100) / 360.0;
                    std::cout << pi.describe << " convert value " << pi.value << std::endl;
                }
                // TODO: 清空 protocol_ 后重新添加
                protocol_[cmd] = pi;
            }
        }

        return true;
    }

    bool tryLoadConfig(const std::string& path) {
        if (isNeedRead()) {
            // 文件修改过，重新读取
            if (loadConfig(path)) {
                std::cout << "JSON updated: " << config_.dump() << std::endl;
                return true;
            } else {
                std::cout << "Failed to parse JSON, keeping previous config." << std::endl;
                return false;
            }
        }
        return false;
    }

    void loadThread() {
        while (running_) {
            tryLoadConfig(jsonPath_);
            // std::cout << "loadThread" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    void recvThread() {
        while (running_) {
            struct can_frame frame{};
            int nbytes = read(sock_, &frame, sizeof(frame));
            if (nbytes > 0) {
                std::lock_guard<std::mutex> lock(mtx_);
                frameQueue_.push(frame);
                cv_.notify_one();
            }
        }
    }

    void processThread() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_real_distribution<float> dist(0.0, 100.0);

        while (running_) {
            std::unique_lock<std::mutex> lock(mtx_);
            cv_.wait(lock, [&]{ return !frameQueue_.empty() || !running_; });
            if (!running_) break;

            auto frame = frameQueue_.front();
            frameQueue_.pop();
            lock.unlock();

            // 匹配 ID
            if (frame.can_id != motorId_) continue;

            char buf[3];
            snprintf(buf, sizeof(buf), "%02X", frame.data[0]);
            std::string cmd(buf);

            std::unique_lock<std::mutex> l(protocol_mtx_);
            auto it = protocol_.find(cmd);
            if (it != protocol_.end()) {
                ProtocolItem item = it->second;
                l.unlock();
                sendResponse(cmd, item);
            } else {
                std::cout << "unknown id: " << cmd << std::endl;
            }
        }
    }

    void sendResponse(const std::string& cmd, const ProtocolItem pi) {
        struct can_frame tx{};
        tx.can_id = motorId_;
        tx.can_dlc = 5; // CMD + float(4字节)

        // 第一个字节放命令
        tx.data[0] = static_cast<uint8_t>(std::stoi(cmd, nullptr, 16));

        int32_t pos = static_cast<int32_t>(pi.value);
        std::vector<uint8_t> data = {
            static_cast<uint8_t>(pos & 0xFF),
            static_cast<uint8_t>((pos >> 8) & 0xFF),
            static_cast<uint8_t>((pos >> 16) & 0xFF),
            static_cast<uint8_t>((pos >> 24) & 0xFF)
        };
        for (size_t i = 0; i < data.size(); ++i) {
            tx.data[i + 1] = data[i];
        }

        if (write(sock_, &tx, sizeof(tx)) != sizeof(tx)) {
            perror("write");
        } else {
            std::cout << "Sent response for cmd=" << cmd
                      << " value=" << pi.value << std::endl;
        }
    }

private:
    std::string ifname_;
    int sock_{-1};
    int motorId_{0};
    nlohmann::json config_;
    std::unordered_map<std::string, ProtocolItem> protocol_;

    std::queue<struct can_frame> frameQueue_;
    std::mutex mtx_;
    std::condition_variable cv_;
    std::atomic<bool> running_{false};
    std::thread tRecv_;
    std::thread tProc_;

    time_t lastModifyTime;
    std::mutex protocol_mtx_;
    std::thread tLoad_;
};

class VirtualImu : public JsonHelper {
public:
    VirtualImu(const std::string& jsonPath)
        : fd_(-1), running_(false) {
        jsonPath_ = jsonPath;
        loadConfig();
    };

    ~VirtualImu() {
        stop();
    };

    bool start() {
        if (!openSerial()) return false;
        running_ = true;
        sendThread_ = std::thread(&VirtualImu::sendLoop, this);
        return true;
    };

    void stop() {
        running_ = false;
        if (sendThread_.joinable())
            sendThread_.join();
        closeSerial();
    };

private:
    void loadConfig() {
        json j;
        safeReadJson(jsonPath_, j);

        std::lock_guard<std::mutex> lock(configMutex_);
        port_ = j.value("port", "/tmp/ttyV0");
        baudrate_ = j.value("baud", 115200);
        frequencyHz_ = j.value("frequency_hz", 10);
        loopMessages_ = j.value("loop_messages", true);

        messages_.clear();
        for (auto& msg : j["messages"]) {
            Message m;
            m.describe = msg.value("describe", "");
            m.data = msg.value("data", 0);
            m.length = msg.value("length", 2);
            messages_.push_back(m);
        }

        std::cout << "Config loaded: " << messages_.size() 
                << " messages, freq=" << frequencyHz_ << "Hz" << std::endl;
    };

    void sendLoop() {
        while (running_) {
            if (isNeedRead()) {
                std::cout << "Config file modified, reloading..." << std::endl;
                loadConfig();
            }

            std::vector<uint8_t> frame;
            {
                std::lock_guard<std::mutex> lock(configMutex_);
                frame = buildFrame();
            }

            sendData(frame);

            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / frequencyHz_));
        }
    };

    bool openSerial() {
        fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            std::cerr << "Failed to open " << port_ << std::endl;
            return false;
        }

        struct termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            std::cerr << "Error from tcgetattr" << std::endl;
            return false;
        }

        cfsetospeed(&tty, baudrate_);
        cfsetispeed(&tty, baudrate_);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag = 0;
        tty.c_oflag = 0;
        tty.c_lflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            std::cerr << "Error from tcsetattr" << std::endl;
            return false;
        }

        std::cout << "Serial port " << port_ << " opened." << std::endl;
        return true;
    };

    void closeSerial() {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    };

    void sendData(const std::vector<uint8_t>& data) {
        if (fd_ < 0) return;
        write(fd_, data.data(), data.size());
        tcdrain(fd_);
    };

    std::vector<uint8_t> buildFrame() {
        std::vector<uint8_t> frame;
        int count = 0;
        frame.insert(frame.end(), {0xAA, 0x55, 0, 0, 0, 0});
        for (const auto& msg : messages_) {
            uint32_t val = msg.data;
            for (int i = 0; i < msg.length; ++i) {
                count++;
                frame.push_back(static_cast<uint8_t>(val & 0xFF)); // 小端序
                val >>= 8;
            }
        }
        frame.insert(frame.end(), {0, 0, 0, 0});
        // std::cout << "frame count: " << count + 10 << std::endl;
        return frame;
    };

private:
    std::string port_;
    int baudrate_;
    int frequencyHz_;
    bool loopMessages_;

    struct Message {
        std::string describe;
        uint32_t data;
        int length;
    };
    std::vector<Message> messages_;

    int fd_; // 串口文件描述符
    std::thread sendThread_;
    std::atomic<bool> running_;
    std::mutex configMutex_;
};
