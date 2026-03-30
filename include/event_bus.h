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
#include <sstream>
class EventBus
{
public:
    EventBus() { AERROR << "bus construct, this" << this << "global/main bus"; };

    template <typename EventT>
    using Callback = std::function<void(const EventT)>;

    template <typename EventT>
    void subscribe(const std::string &event, Callback<EventT> cb)
    {
        AINFO << "订阅事件: " << event
              << "this " << this
              << "typename " << typeid(EventT).name();
        std::lock_guard<std::mutex> lock(mutex_);
        listeners[event].push_back({std::type_index(typeid(EventT)),
                                    [cb](const std::any e)
                                    { cb(std::any_cast<const EventT>(e)); }});
        AERROR << event << " " << listeners[event].size();
    }

    template <typename EventT>
    void publish(const std::string &event, EventT data)
    {
        AINFO << "开始发布事件: " << event;
        std::vector<CallbackWrapper> to_call;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = listeners.find(event);
            if (it == listeners.end() || it->second.empty())
            {
                AINFO << "发布事件: " << event << " 没有监听者";
                return;
            }

            std::type_index publish_type = std::type_index(typeid(EventT));
            for (const auto &wrapper : it->second){
                if (wrapper.type == publish_type)
                {
                    to_call.push_back(wrapper);
                }else{
                    AINFO<< "发布事件: " << event << " 存在监听者但类型不匹配，监听者类型=" << wrapper.type.name() << " 发布类型=" << publish_type.name();
                }
                
            }
            AINFO<<"thsi="<<this <<"事件："<<event<<"匹配："<<to_call.size()<<"解锁";
        }

        for(const auto& wrapper : to_call){
            wrapper.fn(data);
        }
      
    }

private:
    struct CallbackWrapper
    {
        std::type_index type;
        std::function<void(const std::any)> fn;
    };

    std::unordered_map<std::string, std::vector<CallbackWrapper>> listeners;
    std::mutex mutex_;
};
