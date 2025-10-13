#pragma once
#include <functional>
#include <unordered_map>
#include <vector>
#include <string>
#include <log.h>
#include <nlohmann/json.hpp>

class EventBus {
public:
    EventBus() { AERROR << "bus construct"; };
    using json = nlohmann::json;
    using Callback = std::function<void(const json)>;

    void subscribe(const std::string& event, Callback cb) {
        listeners[event].emplace_back(std::move(cb));
        AERROR << event << " " << listeners[event].size();
    }

    void publish(const std::string& event, json data) {
        AERROR << event << " " << listeners[event].size();
        for (auto& cb : listeners[event]) {
            cb(data);
        }
    }

private:
    std::unordered_map<std::string, std::vector<Callback>> listeners;
};
