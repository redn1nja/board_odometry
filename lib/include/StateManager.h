// StateManager.h
#pragma once
#include <map>
#include <vector>
#include <mutex>
#include <memory>
#include <functional>
#include "types.h"

namespace vision {

class StateManager {
public:
    // Singleton access
    static StateManager& getInstance() {
        static StateManager instance;
        return instance;
    }

    // State keys
    enum class StateKey {
        RC_CHANNELS,
        ATTITUDE,
        DETECTED_OBJECTS,
        TRACKING_INFO,
        TELEMETRY_DATA
    };

    // Generic state setter
    template <typename T>
    void setState(StateKey key, const T& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = states_.find(static_cast<int>(key));
        if (it != states_.end()) {
            auto typedState = std::dynamic_pointer_cast<TypedState<T>>(it->second);
            if (typedState && typedState->value == value) {
                return;  // Value hasn't changed, no need to update
            }
        }
        states_[static_cast<int>(key)] = std::make_shared<TypedState<T>>(value);
        mutex_.unlock();
        notifyObservers(key);
    }

    // Generic state getter
    template <typename T>
    T getState(StateKey key, const T& defaultValue = T()) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = states_.find(static_cast<int>(key));
        if (it != states_.end()) {
            auto typedState = std::dynamic_pointer_cast<TypedState<T>>(it->second);
            if (typedState) {
                return typedState->value;
            }
        }
        return defaultValue;
    }

    // Observer registration
    using ObserverCallback = std::function<void(StateKey)>;
    int addObserver(StateKey key, ObserverCallback callback) {
        std::lock_guard<std::mutex> lock(mutex_);
        int id = next_observer_id_++;
        observers_[static_cast<int>(key)][id] = callback;
        return id;
    }

    // Observer removal
    void removeObserver(StateKey key, int observerId) {
        std::lock_guard<std::mutex> lock(mutex_);
        observers_[static_cast<int>(key)].erase(observerId);
    }

private:
    StateManager() = default;
    ~StateManager() = default;
    StateManager(const StateManager&) = delete;
    StateManager& operator=(const StateManager&) = delete;

    // Base state type for polymorphism
    struct BaseState {
        virtual ~BaseState() = default;
    };

    // Typed state container
    template <typename T>
    struct TypedState : BaseState {
        explicit TypedState(const T& v) : value(v) {}
        T value;
    };

    void notifyObservers(StateKey key) {
        auto& keyObservers = observers_[static_cast<int>(key)];
        for (const auto& observer : keyObservers) {
            observer.second(key);
        }
    }

    std::map<int, std::shared_ptr<BaseState>> states_;
    std::map<int, std::map<int, ObserverCallback>> observers_;
    std::mutex mutex_;
    int next_observer_id_ = 0;
};

} // namespace vision