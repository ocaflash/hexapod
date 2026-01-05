/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

class CCallbackTimer {
   public:
    CCallbackTimer() : isRunning_(false) {
    }

    void start(uint32_t intervalMilliseconds, std::function<void()> callback, bool singleShot = false) {
        stop();
        {
            std::lock_guard<std::mutex> lk(mtx_);
            isRunning_ = true;
        }
        timerThread_ = std::thread([this, intervalMilliseconds, callback, singleShot]() {
            std::unique_lock<std::mutex> lk(mtx_);
            // wait_for wakes early on stop(), so we don't have detached threads lingering.
            while (isRunning_) {
                if (cv_.wait_for(lk, std::chrono::milliseconds(intervalMilliseconds),
                                 [this]() { return !isRunning_; })) {
                    break;
                }
                // execute callback outside of the lock to avoid deadlocks
                lk.unlock();
                callback();
                lk.lock();
                if (singleShot) {
                    isRunning_ = false;
                }
            }
        });
    }

    void startWithoutCallback(uint32_t intervalMilliseconds) {
        start(intervalMilliseconds, []() {}, true);
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            isRunning_ = false;
        }
        cv_.notify_all();
        if (timerThread_.joinable()) timerThread_.join();
    }

    bool isRunning() {
        return isRunning_;
    }

    ~CCallbackTimer() {
        stop();
    }

   private:
    std::thread timerThread_;
    std::atomic<bool> isRunning_;
    std::mutex mtx_;
    std::condition_variable cv_;
};
