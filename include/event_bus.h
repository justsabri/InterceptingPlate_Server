#pragma once
#include <functional>
#include <unordered_map>
#include <vector>
#include <string>
#include <log.h>
// #include <nlohmann/json.hpp>
#include <any>
#include <typeindex>
#include <mutex>

class EventBus {
public:
    EventBus() { AERROR << "bus construct"; };

    template<typename EventT>
    using Callback = std::function<void(const EventT)>;

    template<typename EventT>
    void subscribe(const std::string& event, Callback<EventT> cb) {
        std::lock_guard<std::mutex> lock(mutex_);
        listeners[event].push_back({std::type_index(typeid(EventT)),
                       [cb](const std::any e){ cb(std::any_cast<const EventT>(e)); }});
        AERROR << event << " " << listeners[event].size();
    }

    template<typename EventT>
    void publish(const std::string& event, EventT data) {
        AERROR << event << " " << listeners[event].size();
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = listeners.find(event);
        if (it != listeners.end()) {
            for (auto& [type, fn] : it->second) {
                if (type == std::type_index(typeid(EventT))) {
                    fn(data);
                }
            }
        }
    }

private:
    struct CallbackWrapper {
        std::type_index type;
        std::function<void(const std::any)> fn;
    };

    std::unordered_map<std::string, std::vector<CallbackWrapper>> listeners;
    std::mutex mutex_;
};
