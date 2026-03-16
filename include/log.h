#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <memory>
#include <chrono>
#include <iomanip>

namespace qclcpp {
#ifdef LOG_LEVEL_ERROR
#define LOG_LEVEL 1
#elif defined(LOG_LEVEL_WARN)
#define LOG_LEVEL 2
#elif defined(LOG_LEVEL_INFO)
#define LOG_LEVEL 3
#elif defined(LOG_LEVEL_DEBUG)
#define LOG_LEVEL 4
#else
#define LOG_LEVEL 1 // 默认为 error
#endif

#define log_error(fmt, ...) if (LOG_LEVEL >= 1) log_message("Error", __func__, __LINE__, fmt, ##__VA_ARGS__)
#define log_warn(fmt, ...) if (LOG_LEVEL >= 2) log_message("Warn", __func__, __LINE__, fmt, ##__VA_ARGS__)
#define log_info(fmt, ...) if (LOG_LEVEL >= 3) log_message("Info", __func__, __LINE__, fmt, ##__VA_ARGS__)
#define log_debug(fmt, ...) if (LOG_LEVEL >= 4) log_message("Debug", __func__, __LINE__, fmt, ##__VA_ARGS__)

// 格式化字符串的帮助函数
template<typename... Args>
std::string format(const std::string& fmt, Args... args)
{
    int size = snprintf(nullptr, 0, fmt.c_str(), args...) + 1; // 包括额外的 '\0'
    if (size <= 0) {
        throw std::runtime_error("Error during formatting.");
    }
    std::unique_ptr<char[]> buf(new char[size]);
    snprintf(buf.get(), size, fmt.c_str(), args...);
    return std::string(buf.get(), buf.get() + size - 1); // 去掉额外的 '\0'
}

// 没有参数的情况
inline std::string format(const std::string& fmt)
{
    return fmt;
}

inline uint64_t get_timestamp_ms() {
    // 获取当前时间点
    auto now = std::chrono::system_clock::now();
    // 转换为自纪元以来的毫秒数
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    return timestamp;
}

// 可变模板参数
template<typename... Args>
void log_message(const std::string& level, const std::string& func, int line, const std::string& fmt, Args... args)
{
    std::ostringstream oss;
    // Format timestamp to human-readable format
    auto timestamp = get_timestamp_ms();
    time_t seconds = timestamp / 1000;
    int milliseconds = timestamp % 1000;

    char time_buf[32];
    struct tm timeinfo;
    #ifdef _WIN32
    localtime_s(&timeinfo, &seconds);
    #else
    localtime_r(&seconds, &timeinfo);
    #endif
    strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);

    oss << "[" << time_buf << "." << std::setw(3) << std::setfill('0') << milliseconds << "] "
        << "[" << level << "] "
        << "[" << func << ":" << line << "] "
        << format(fmt, args...);
    std::cerr << oss.str() << std::endl;
}
}

#ifdef LOG_TEST
using namespace qclcpp;
int main()
{
    int error_code = 404;
    std::string error_message = "Not Found";

    log_error("Error code: %d, message: %s", error_code, error_message.c_str());
    log_debug("This is a debug message with a number: %d", 42);
    log_warn("This is a warning with a float: %.2f", 3.14159);
    std::cout << "\n";

    if (LOG_LEVEL) {
        std::cout << "LOG_LEVEL=" << LOG_LEVEL << "\n";
    }

    log_warn("This is a warning without args");
    log_warn("This is a warning without one args: %d", 44);

    std::string name{"taotao"};
    log_info("hi %s", name.c_str());
    log_info("socket is not opened");
    log_error(name);
    log_error("hi " + name + " is cute");

    return 0;
}
#endif

/*
g++ -x c++ log.h -DLOG_TEST
*/