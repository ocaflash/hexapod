/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

using namespace std::chrono_literals;

class CCallbackTimer {
   public:
    CCallbackTimer() : isRunning_(false) {
    }

    void start(std::chrono::milliseconds intervalMilliseconds, std::function<void()> callback,
               bool singleShot = false) {
        stop();
        {
            std::lock_guard<std::mutex> lk(mtx_);
            isRunning_ = true;
        }
        timerThread_ = std::thread([this, intervalMilliseconds, callback, singleShot]() {
            std::unique_lock<std::mutex> lk(mtx_);
            while (isRunning_) {
                if (cv_.wait_for(lk, intervalMilliseconds, [this]() { return !isRunning_; })) {
                    break;
                }
                lk.unlock();
                callback();
                lk.lock();
                if (singleShot) isRunning_ = false;
            }
        });
    }

    void start(uint32_t intervalMilliseconds, std::function<void()> callback, bool singleShot = false) {
        start(std::chrono::milliseconds(intervalMilliseconds), callback, singleShot);
    }
    void startWithoutCallback(std::chrono::milliseconds intervalMilliseconds) {
        start(intervalMilliseconds, []() {}, true);
    }
    void startWithoutCallback(uint32_t intervalMilliseconds) {
        startWithoutCallback(std::chrono::milliseconds(intervalMilliseconds));
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

// int main() {
//     CCallbackTimer timer;

//     // Callback-Funktion
//     auto callback = []() { std::cout << "CCallbackTimer triggered!" << std::endl; };

//     // CCallbackTimer starten
//     std::cout << "Starting timer..." << std::endl;
//     timer.start(1000, callback);  // 1 Sekunde (1000 Millisekunden)

//     // CCallbackTimer nach 5 Sekunden stoppen
//     std::this_thread::sleep_for(std::chrono::seconds(5));
//     std::cout << "Stopping timer..." << std::endl;
//     timer.stop();

//     return 0;
// }
