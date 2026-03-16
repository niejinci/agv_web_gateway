#ifndef PERIODIC_TASK_H
#define PERIODIC_TASK_H

/**
 * @file PeriodicTask.h
 * @author niejinci
 * @brief 一个通用的周期性任务类，使用asio的定时器实现
 *
 * 创建了一个新的 periodicTask.h 文件来封装定时任务的通用逻辑。
 * 这个类管理 asio::steady_timer 和一个 is_running_ 状态标志。
 * start(interval) 方法负责启动定时器，并支持重复调用（如果已经在运行则直接返回）。
 * stop() 方法负责安全地取消定时器。
 * 构造函数接收一个 TaskAction（即一个 std::function<void()>)，这正是策略模式的体现——将要执行的具体算法（任务）作为参数注入。
 * 这样，Client 类中就不需要为每个定时任务重复编写定时器管理代码，只需创建 PeriodicTask 实例并传入相应的任务函数即可。
 *
 * @version 0.1
 * @date 2025-09-27
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "log.h"
#include <asio.hpp>
#include <functional>

namespace qclcpp {

class PeriodicTask {
public:
    using TaskAction = std::function<void()>;

    PeriodicTask(asio::io_context& io_context, std::string task_name, TaskAction action)
        : timer_(io_context), task_name_(std::move(task_name)), action_(std::move(action)) {
            log_info("construct %s", task_name_.c_str());
        }

    void start(asio::chrono::milliseconds interval) {
        if (is_running_) {
            log_info("%s is already running", task_name_.c_str());
            return;
        }
        log_info("start %s", task_name_.c_str());
        interval_ = interval;
        is_running_ = true;
        // 立即执行一次
        action_();
        // 然后启动定时器
        timer_.expires_after(interval_);
        timer_.async_wait([this](const asio::error_code& ec) {
            schedule_next(ec);
        });
    }

    void stop() {
        if (is_running_) {
            is_running_ = false;
            timer_.cancel();
            log_info("%s timer canceled", task_name_.c_str());
        } else {
            log_info("%s is not running, no need to cancel", task_name_.c_str());
        }
    }

    bool is_running() const {
        return is_running_;
    }

    void set_interval(asio::chrono::milliseconds interval) {
        interval_ = interval;
    }

private:
    void schedule_next(const asio::error_code& ec) {
        if (ec == asio::error::operation_aborted) {
            // 任务被 stop() 取消
            log_info("%s timer was canceled by stop()", task_name_.c_str());
            is_running_ = false;
            return;
        }

        if (!is_running_) {
            // 标志位被设置为停止
            return;
        }

        if (ec) {
            log_error("%s timer error: %s", task_name_.c_str(), ec.message().c_str());
            // 出现错误时可以选择停止或继续，这里选择继续
        }

        action_();

        timer_.expires_after(interval_);
        timer_.async_wait([this](const asio::error_code& ec) {
            schedule_next(ec);
        });
    }

    asio::steady_timer timer_;
    std::string task_name_;
    TaskAction action_;
    bool is_running_ = false;
    asio::chrono::milliseconds interval_{1000};
};

} // namespace qclcpp

#endif // PERIODIC_TASK_H