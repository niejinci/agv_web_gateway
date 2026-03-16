#include "client.h"
#include "SimpleIni.h"
#include <fstream>
#include <iostream>
#include <random>
#include <array>
#include <iomanip>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include "nlohmann/json.hpp"
#include "log.h"
#include <thread>
#include <cstdio>
#include <regex>
#include <algorithm>

using json = nlohmann::json;
using namespace nlohmann::literals::json_literals;

namespace qclcpp {

/**
 * @brief 创建Client实例的工厂方法
 *
 * 该静态方法创建并返回Client类的智能指针实例，使用共享指针管理对象生命周期。
 * 通过此工厂方法可以隐藏构造函数细节，提供统一的对象创建接口。
 *
 * @param io_context ASIO库的IO上下文对象，用于处理异步I/O操作
 * @return std::shared_ptr<Client> 指向新创建的Client实例的智能指针
 */
std::shared_ptr<Client> Client::create(asio::io_context& io_context, const std::string& requestname2cmd_file)
{
    return std::shared_ptr<Client>(new Client(io_context, requestname2cmd_file));
}

void Client::default_switch_callback(bool success)
{
    if (success) {
        log_info("Successfully switched to the new server.");
    } else {
        log_warn("Failed to switch to the new server.");
    }
}

void Client::default_disconnect_callback(const std::string& error)
{
    log_warn(error);
}

void Client::heart_beat_callback(const std::string& reply)
{
    json jreply = json::parse(reply, nullptr, false);
    if (jreply.is_discarded() || !jreply.is_object() || jreply.value("data", "") != "Pong") {
        log_warn("not receive valid heartbeat reply");
        // 解析响应数据失败，说明服务端出问题了，发起重连
        connect_lambda(current_server_ip_, current_server_port_);
    } else {
        // log_debug("receive heartbeat reply: %s", jreply.dump().c_str());
    }
}

void Client::heart_beat(const asio::error_code& ec)
{
    if (ec == asio::error::operation_aborted) {
        log_info("hb timer canceled: %s", ec.message().c_str());
        heart_beat_running_ = false;
        return;
    }
    heart_beat_running_ = true;
    if (!current_server_ip_.empty() && !current_server_port_.empty()) {
        if (socket_.is_open()) {
            // 客户端启动过了连接, 那么就开始发送心跳
            send_request("HEART_BEAT", R"({"data": "Ping"})");
        } else if (!disconnect_by_user_) {
            // 如果不是用户触发的断连，那么就自动重连
            connect_lambda(current_server_ip_, current_server_port_);
        }
    } else {
        log_debug("Has Not Connected to the Server Yet");
    }
    heartbeat_timer_->expires_after(asio::chrono::seconds(19));
    heartbeat_timer_->async_wait([this](const asio::error_code& ec) {
        heart_beat(ec);
    });
}

void Client::stop_heart_beat()
{
    if (heartbeat_timer_) {
        heartbeat_timer_->cancel();
    }
}

void Client::restart_heart_beat()
{
    // 防止重复启动
    if (!heart_beat_running_) {
        log_info("restart hb timer");
        automatic_reconnect();
    } else {
        log_info("hb timer is already running");
    }
}

void Client::automatic_reconnect()
{
    register_handler("HEART_BEAT", std::bind(&Client::heart_beat_callback, this, std::placeholders::_1));
    if (!heartbeat_timer_) {
        heartbeat_timer_ = std::make_shared<asio::steady_timer>(io_context_);
    }
    heart_beat(asio::error_code());
}

Client::Client(asio::io_context& io_context, const std::string& requestname2cmd_file_arg)
    : io_context_(io_context)
    , socket_(io_context)
    , write_in_progress_(false)
    , pointcloud_socket_(io_context)  // 初始化点云专用套接字
    , get_3dcamera_pointcloud_task_(io_context, "get_3dcamera_pointcloud", [this]() { this->send_request_on_pointcloud_socket("GET_CAMERA_POINT_CLOUD", ""); })
    , get_robot_state_task_(io_context, "get_robot_state", [this]() { this->send_request_on_pointcloud_socket("GET_ROBOT_STATE", ""); })
    , get_model_polygon_task_(io_context, "get_model_polygon", [this]() { this->send_request_on_pointcloud_socket("GET_MODEL_POLYGON", ""); })
    , get_obst_pcl_task_(io_context, "get_obst_pcl", [this]() { this->send_request_on_pointcloud_socket("GET_OBST_PCL", ""); })
    , get_obst_polygon_task_(io_context, "get_obst_polygon", [this]() { this->send_request_on_pointcloud_socket("GET_OBST_POLYGON", ""); })
    , get_scan2pointcloud_task_(io_context, "get_scan2pointcloud", [this]() { this->send_request_on_pointcloud_socket("GET_SCAN2POINTCLOUD", ""); })
    , get_qr_camera_data_task_(io_context, "get_qr_camera_data", [this]() { this->send_request_on_pointcloud_socket("GET_QR_CAMERA_DATA", ""); })
    , get_point_cloud_task_(io_context, "get_point_cloud", [this]() { this->send_request_on_pointcloud_socket("GET_POINT_CLOUD", ""); })
    , get_agv_position_task_(io_context, "get_agv_position", [this]() { this->send_request_on_pointcloud_socket("LOCALIZATION_QUALITY", ""); })
{
    bool ret = true;
    std::string config_file = requestname2cmd_file_arg.empty() ? requestname2cmd_file : requestname2cmd_file_arg;
    ret = load_file(config_file);
    if (!ret) {
        std::string error_msg = "failed to load requestname2cmd_file: " + config_file;
        log_error("%s", error_msg.c_str());
        throw std::runtime_error(error_msg);
    }

    automatic_reconnect();

    connect_lambda = [this](const std::string& host, const std::string& port) {
        connect(host, port, [](bool success) {
            if (success) {
                log_debug("reconnect success");
            } else {
                log_warn("reconnect failed");
            }
        });
    };
}

// 辅助函数，用于处理延迟重连逻辑
void Client::reconnect_pointcloud_after_delay()
{
    if (!disconnect_by_user_ && main_connected_) {
        // 创建一个定时器，生命周期由 shared_ptr 管理
        auto timer = std::make_shared<asio::steady_timer>(io_context_);

        // 设置定时器 1 秒后到期
        timer->expires_after(asio::chrono::seconds(1));

        // 异步等待定时器，将重连逻辑放入回调
        timer->async_wait([this, timer](const asio::error_code& ec) {
            // 如果定时器不是被取消的
            if (!ec) {
                log_info("Timer expired, attempting to reconnect pointcloud socket...");
                connect_pointcloud_socket(current_server_ip_, current_server_port_,
                    [](bool success) {
                        log_info("Pointcloud reconnect attempt %s.", success ? "succeeded" : "failed");
                    });
            }
        });
    }
}

// 连接点云专用套接字
void Client::connect_pointcloud_socket(const std::string& host, const std::string& port, std::function<void(bool)> callback)
{
    std::unique_lock<std::mutex> lock(pointcloud_connection_mutex_);
    // 关闭现有的连接
    if (pointcloud_socket_.is_open()) {
        log_info("close pointcloud_socket_ before connect");
        std::error_code ec;
        pointcloud_socket_.shutdown(asio::ip::tcp::socket::shutdown_both, ec);
        pointcloud_socket_.close(ec);
    }

    asio::ip::tcp::resolver resolver(io_context_);
    auto endpoints = resolver.resolve(host, port);

    asio::async_connect(pointcloud_socket_, endpoints, [this, callback](const std::error_code& ec, const asio::ip::tcp::endpoint& endpoint) {
        callback(!ec);
        (void)endpoint; //消除编译告警

        if (!ec) {
            // 设置点云连接的缓冲区大小 - 比常规连接更大
            asio::socket_base::receive_buffer_size option(1048576); // 1MB
            pointcloud_socket_.set_option(option);

            // 禁用Nagle算法，减少传输延迟
            asio::ip::tcp::no_delay nodelay_option(true);
            pointcloud_socket_.set_option(nodelay_option);

            pointcloud_connected_ = true;
            do_read_pointcloud();
        } else {
            log_error("pointcloud connection failed: %s, value=%d", ec.message().c_str(), ec.value());
            pointcloud_connected_ = false;
        }
    });
}

/**
 * @brief 校验端口
 *  规则：非空、纯数字、范围 0-65535
 * @param portStr 字符串格式的端口号
 * @return true 端口合法
 * @return false 端口不合法
 */
bool Client::is_valid_port(const std::string& portStr)
{
    if (portStr.empty()) {
        return false;
    }

    // 检查是否全为数字
    if (!std::all_of(portStr.begin(), portStr.end(), ::isdigit)) {
        return false;
    }

    // 检查数字范围
    try {
        int port = std::stoi(portStr);
        return port >= 0 && port <= 65535;
    } catch (...) {
        // 捕获 std::stoi 可能抛出的异常（如超长数字导致的 out_of_range）
        return false;
    }
}

/**
 * @brief 校验 IPv4
 *  规则：使用正则匹配 standard IPv4 格式 (0-255).(0-255).(0-255).(0-255)
 * @param ip 字符串格式的ip地址
 * @return true ip合法
 * @return false ip不合法
 */
bool Client::is_valid_ipv4(const std::string& ip)
{
    // 使用 static const 避免每次调用函数都重新编译正则表达式，提高性能
    static const std::regex ipv4Regex(
        R"(^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$)"
    );
    return std::regex_match(ip, ipv4Regex);
}

/**
 * @brief 连接到指定的服务器。
 *
 * 此函数尝试建立与服务器的TCP连接。
 * 首先关闭任何现有连接，然后异步连接到指定的主机和端口。
 * 连接成功后，会启动数据读取并尝试建立点云专用连接。
 * 无论点云连接是否成功，只要主连接成功就会认为整体连接成功。
 *
 * @param host 服务器主机名或IP地址
 * @param port 服务器端口号
 * @param callback 连接完成后的回调函数，参数为连接是否成功
 */
void Client::connect(const std::string& host, const std::string& port, std::function<void(bool)> callback)
{
    // 1. 校验 Port (通常计算成本较低，先校验)
    if (!is_valid_port(port)) {
        std::cerr << "[Error] Invalid port: " << port << std::endl;
        if (callback) callback(false);
        return;
    }

    // 2. 校验 Host (IPv4)
    if (!is_valid_ipv4(host)) {
        std::cerr << "[Error] Invalid IPv4 address: " << host << std::endl;
        if (callback) callback(false);
        return;
    }

    // 关闭现有的连接
    close_socket();
    clear_write_status();
    std::this_thread::sleep_for(asio::chrono::milliseconds(100));

    current_server_ip_ = host;
    current_server_port_ = port;

    // 设置连接超时定时器
    auto timer = std::make_shared<asio::steady_timer>(io_context_);
    timer->expires_after(std::chrono::seconds(3));
    timer->async_wait([this](const asio::error_code& ec) {
    if (!ec) {
            socket_.cancel();
        }
    });

    asio::ip::tcp::resolver resolver(io_context_);
    auto endpoints = resolver.resolve(host, port);
    asio::async_connect(socket_, endpoints, [this, callback, timer](const std::error_code& ec, const asio::ip::tcp::endpoint& endpoint) {
        callback(!ec);
        main_connected_ = !ec;
        (void)endpoint; //消除编译告警
        timer->cancel(); // 取消定时器
        if (!ec) {
            do_read();

            // 【优化点】主连接成功后，立即发起数据通道的连接
            log_info("Main connection successful. Establishing data channel (pointcloud socket)...");
            connect_pointcloud_socket(current_server_ip_, current_server_port_, [](bool success) {
                if (success) {
                    log_info("Data channel connected successfully.");
                } else {
                    log_warn("Failed to connect data channel. It will be retried on next data request failure.");
                }
            });
        } else {
            log_error("connect failed: %s, value=%d", ec.message().c_str(), ec.value());
            close_socket();
            clear_write_status();
        }
    });

    disconnect_by_user_ = false;
}

void Client::close_pointcloud_socket()
{
    cancel_get_agv_position();
    cancel_get_point_cloud();
    cancel_get_3dcamera_pointcloud();
    cancel_get_qr_camera_data();
    cancel_get_scan2pointcloud();
    cancel_get_obst_polygon();
    cancel_get_obst_pcl();
    cancel_get_model_polygon();

    // 关闭点云专用连接
    if (pointcloud_socket_.is_open()) {
        std::error_code ec;
        pointcloud_socket_.shutdown(asio::ip::tcp::socket::shutdown_both, ec);
        pointcloud_socket_.close(ec);
        pointcloud_connected_ = false;
        log_info("disconnect pointclout socket");
    }
}

void Client::close_socket(std::function<void(const std::string& error)> callback)
{
    cancel_get_sysinfo();

    // 关闭 socket 以断开与服务器的连接
    if (socket_.is_open()) {
        std::error_code ec;
        // 禁用套接字上的发送或接收操作。
        socket_.shutdown(asio::ip::tcp::socket::shutdown_both, ec);
        if (ec) {
            callback(ec.message());
        }

        // 此函数用于关闭套接字。任何异步发送、接收或连接操作将立即被取消，并将以 asio::error::operation_aborted 错误完成。
        socket_.close(ec);
        if (ec) {
            callback(ec.message());
        }
    } else {
        log_info("socket is not opened, no need close");
    }
    main_connected_ = false;

    close_pointcloud_socket();
}

void Client::clear_write_status()
{
    // 清空写入队列, 重置写入标志
    std::lock_guard<std::mutex> lock(write_queue_mutex_);
    write_in_progress_ = false;
    while (!write_queue_.empty()) {
        write_queue_.pop();
    }
}

void Client::disconnect(std::function<void(const std::string& error)> callback)
{
    close_socket(callback);
    clear_write_status();

    disconnect_by_user_ = true;
    current_server_ip_.clear();
}


std::string Client::create_packet(const std::string& request_name, const std::string& uuid, const std::string& msg)
{
    // packet: sync_field(2byte) + cmd(2byte) + length(2byte) + uuid(36byte) + msg({length}-36)

    // 根据请求名称找到请求对应的命令字
    if (requestname2cmd_.find(request_name) == requestname2cmd_.end()) {
        log_error("not find [%s]'s cmd, please configure it first", request_name.c_str());
        return "";
    }
    uint16_t cmd = requestname2cmd_[request_name];
    std::ostringstream oss;
    // 长度字段只包含数据部分的长度
    uint16_t length = uuid.size() + msg.size();
    oss << std::hex << std::setw(4) << std::setfill('0') << sync_field_
        << std::setw(4) << cmd
        << std::setw(4) << length
        << uuid
        << msg;

    return oss.str();
}

void Client::uuid_to_handler_(const std::string& uuid, ResponseHandler handler)
{
    std::lock_guard<std::mutex> lock(handler_mutex_);
    uuid2handlers_[uuid] = handler;
}

void Client::pointcloud_uuid_to_handler_(const std::string& uuid, ResponseHandler handler)
{
    std::lock_guard<std::mutex> lock(pointcloud_handler_mutex_);
    pointcloud_uuid2handlers_[uuid] = handler;
}

/**
 * @brief 发送请求到服务器
 *
 * 通过传入的命令字找其对应的处理程序，然后发送请求
 *
 * @param request_name string, 对应请求的名字
 * @param request string, 请求数据
 * @param strUuid[optional] string, 请求的唯一标识，如果为空则生成一个
 * @return 如果请求成功加入发送队列，返回true；否则返回false
 *
 * @note 如果套接字未打开或当前有写操作正在进行，函数将返回false
 */
bool Client::send_request(const std::string& request_name, const std::string& request, const std::string& strUuid)
{
    ResponseHandler& handler = requestname2handlers_[request_name];
    if (!socket_.is_open()) {
        log_error("socket is not open, please connect to server first");
        return false;
    }

    // 记录请求，以便在收到响应时调用对应的处理程序
    std::string uuid = strUuid;
    if (uuid.empty()) {
        uuid = generate_uuid_v4();
    }
    uuid_to_handler_(uuid, handler);

    // 创建请求包
    std::string packet = create_packet(request_name, uuid, request);
    if (packet.empty()) {
        log_error("create packet failed");
        return false;
    }

    {
        // 为保护 write_queue_ 的 emplace 操作而加锁
        std::lock_guard<std::mutex> lock(write_queue_mutex_);
        write_queue_.emplace(packet);
    }
    // log_debug("write_queue_.size=%lu, write_in_progress_=%d", write_queue_.size(), write_in_progress_);

    start_next_write();
    return true;
}

void Client::start_next_write()
{
    std::lock_guard<std::mutex> lock(write_queue_mutex_);

    // 如果正在写或队列为空，则直接返回。
    // 这个检查必须在锁内，以防止和 async_write 回调中的状态修改产生竞争。
    if (write_in_progress_ || write_queue_.empty()) {
        return;
    }
    write_in_progress_ = true;

    // 创建一个 shared_ptr 来管理 packet 的生命周期，这是 Asio 的推荐做法
    auto msg = std::make_shared<std::string>(write_queue_.front());

    // buffer 会生成一个 mutable_buffer 对象，把 msg 指向的地址赋值给 mutable_buffer 的 data_ 成员
    asio::async_write(socket_, asio::buffer(*msg),
                        [this, msg](std::error_code ec, std::size_t bytes_transferred) {
                            // 再次加锁来修改队列和状态
                            {
                                std::lock_guard<std::mutex> lock(write_queue_mutex_);
                                write_queue_.pop(); // 移除已发送的请求
                                write_in_progress_ = false;
                            }

                            if (ec) {
                                log_error("message=%s, value=%d", ec.message().c_str(), ec.value());
                                // 有可能是服务端关闭了连接
                                // 断开连接，由心跳机制重新连接
                                close_socket();
                                if (ifile_stream_upload_.is_open()) {
                                    ifile_stream_upload_.close();
                                }

                            } else {
                                // 成功发送后，立即尝试启动下一个写操作
                                // 这会形成一个自我驱动的“写循环”
                                start_next_write();
                            }
                        });
}


/**
 * @brief 生成 UUID v4 格式的唯一标识符
 *
 * 该函数通过结合随机数和当前系统时间戳生成 UUID v4 格式的唯一标识符。
 * 特别解决了在 Qt 程序中 random_device 可能生成相同种子的问题，
 * 通过与系统时间戳异或操作来增强随机性。
 *
 * UUID v4 格式遵循标准规范：8-4-4-4-12 位十六进制数，
 * 其中第三组的第一位固定为 4（版本号），第四组的第一位在 8-11 之间（变体位）。
 *
 * @return std::string 返回格式化的 UUID 字符串，例如 "xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx"
 *         其中 x 是任意十六进制数字，y 是 8-11 范围内的十六进制数字
 */
std::string Client::generate_uuid_v4()
{
    std::random_device rd;
    // 在 windows qt 程序中 random_device 每次生成的种子都是一样的，所以每次生成的 uuid 都是一样的，结合时间戳可以生成不同的种子
    std::mt19937 gen(rd() ^ std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_int_distribution<int> dis(0, 15);
    std::uniform_int_distribution<int> dis2(8, 11);

    std::stringstream ss;
    int i;
    ss << std::hex;
    for (i = 0; i < 8; i++) {
        ss << dis(gen);
    }
    ss << "-";
    for (i = 0; i < 4; i++) {
        ss << dis(gen);
    }
    ss << "-4"; // version 4
    for (i = 0; i < 3; i++) {
        ss << dis(gen);
    }
    ss << "-";
    ss << dis2(gen); // variant bits
    for (i = 0; i < 3; i++) {
        ss << dis(gen);
    }
    ss << "-";
    for (i = 0; i < 12; i++) {
        ss << dis(gen);
    }

    return ss.str();
}

/**
 * @brief 为指定请求名称注册响应处理器
 *
 * 该函数将一个响应处理函数与特定的请求名称关联起来。在注册之前，
 * 会检查请求名称是否已经有对应的处理器（如有则直接返回成功），
 * 以及请求名称是否已配置对应的命令字段（如无则注册失败）。
 *
 * @param request_name 请求名称，用于标识特定类型的请求
 * @param handler 处理该类型请求的响应处理函数
 * @return 注册成功返回true，失败返回false（当请求名称未配置对应命令时）
 *
 * @note 此函数内部使用互斥锁保证线程安全
 */
bool Client::register_handler(const std::string& request_name, ResponseHandler handler)
{
    std::lock_guard<std::mutex> lock(handler_mutex_);
    if (requestname2handlers_.find(request_name) != requestname2handlers_.end()) {
        requestname2handlers_[request_name] = handler;  // 支持更新注册处理程序，刘工反馈同一个请求，会有不同的回调处理
        return true;
    }
    // 根据请求名称找到请求对应的命令字段
    if (requestname2cmd_.find(request_name) == requestname2cmd_.end()) {
        log_error("not find [%s]'s cmd, please configure it first", request_name.c_str());
        return false;
    }
    requestname2handlers_.insert({request_name, handler});
    log_info("register %s success, now requestname2handlers_.size=%ld", request_name.c_str(), requestname2handlers_.size());
    return true;
}

/**
 * @brief 异步读取套接字数据并处理响应
 *
 * 该函数通过 async_read_until 从套接字异步读取数据，直到遇到分隔符"\r\n\r\n"。
 * 数据格式为：uuid|is_binary|data\r\n\r\n，其中：
 * - uuid: 用于标识请求，匹配对应的回调处理函数
 * - is_binary: 为1表示数据是msgpack格式，为0表示普通文本
 * - data: 实际的响应数据
 *
 * 函数会解析响应，查找并调用对应的回调函数。对于文件上传相关的响应，
 * 会进行特殊处理（检查上传进度，在完成时调用上传完成回调）。
 *
 * 处理完一个响应后，函数会递归调用自身以持续读取新的响应。
 */
void Client::do_read()
{
    // 响应数据以 \r\n\r\n 结尾
    // async_read_until 读取的内容会包含分隔符
    log_debug("ready to read");
    asio::async_read_until(socket_, response_, "\r\n\r\n", [this](const std::error_code& ec, std::size_t bytes_transferred) {
        if (!ec) {
            log_debug("read success: %lu, bytes read, response_.size= %lu", bytes_transferred, response_.size());
            // 构造 string 从 streambuf 的开始位置到结束位置
            std::string reply(
                        asio::buffers_begin(response_.data()),
                        asio::buffers_begin(response_.data()) + bytes_transferred
                    );

            // 消耗掉已经处理过的数据
            response_.consume(bytes_transferred);
            // 响应格式为: uuid|is_binary|data\r\n\r\n
            reply = reply.substr(0, reply.size() - 4);  //4 is the length of "\r\n\r\n"
            size_t delim_pos = reply.find('|');
            auto uuid = reply.substr(0, delim_pos);
            log_debug("uuid=%s", uuid.c_str());
            bool is_msgpack = (reply[delim_pos + 1] == '1');
            std::string response;
            if (is_msgpack) {
                response = reply.substr(delim_pos + 3);
                json obj = json::from_msgpack(response);
                response = obj.dump();
            } else {
                response = reply.substr(delim_pos + 3); // 3 is the length of "|0|"
            }

            ResponseHandler handler_to_call;
            bool handler_found = false;
            {
                // 加锁情况下查找和删除 uuid2handlers_ 中的 handler
                std::lock_guard<std::mutex> lock(handler_mutex_);
                auto iter = uuid2handlers_.find(uuid);
                if (iter != uuid2handlers_.end()) {
                    handler_to_call = iter->second;     // 复制 handler
                    handler_found = true;
                    // iter->second(response);

                    // 删除非文件上传相关的 uuid 映射
                    if (uuid != upload_file_uuid_ && uuid != get_file_uuid_ && uuid != push_file_uuid_) {
                        uuid2handlers_.erase(iter);
                    }
                }
            }   // 释放锁

            if (handler_found) {
                // 在锁外面调用 handler，避免死锁，提高性能
                handler_to_call(response);

                // 解析文件上传的返回，如果出错了，取消定时任务
                if (uuid == upload_file_uuid_) {
                    json obj = json::parse(response, nullptr, false);
                    if (obj.is_discarded() || !obj.is_object() || obj["code"] != 0) {
                        log_warn(response.c_str());
                        upload_file_failed_ = true;
                        do_read();
                        return;
                    }

                    // 收到所有文件上传的响应
                    log_debug("receivedFileSize=%d, file_size_=%d", obj["data"].value("receivedFileSize", 0), file_size_);
                    if (obj["data"].value("receivedFileSize", 0) >= file_size_) {
                        upload_handler_(R"({"code": 0, "message": "file upload completed"})");
                    }
                }
            } else {
                log_error("no handler for uuid: %s", uuid.c_str());
            }
            do_read();
        } else {
            log_error("async_read_until failed: %s", ec.message().c_str());
        }
    });
}

bool Client::load_file(const std::string& file_path)
{
    if (file_path.empty()) {
        log_error("requestname2cmd.ini is empty");
        return false;
    }
	CSimpleIniA ini;
	ini.SetUnicode();

	int rc = ini.LoadFile(file_path.c_str());
	if (rc < 0) {
        log_error("LoadFile failed");
        return false;
    };

    auto section = ini.GetSection("config");
    if (!section) {
        log_error("section [config] not found");
        return false;
    }

    for (auto& kv : *section) {
        requestname2cmd_.insert({kv.first.pItem, std::stoi(kv.second, 0, 16)});
    }
    log_info("requestname2cmd_.size=%ld", requestname2cmd_.size());
    return true;
}

/**
 * @brief 获取小车位置信息 api
 *
 * 客户请求该api后，api会创建一个定时器，定时器每隔100ms会向服务器发送一次请求.
 * 如果想在停止获取小车位置时，调用 cancel_get_agv_position() 函数
 * 想再次获取小车位置时，再调用一次 get_agv_position([handler]) 函数
 *
 * @param handler 请求完成后的回调函数
 */
bool Client::get_agv_position(ResponseHandler handler)
{
    return process_request("LOCALIZATION_QUALITY", handler, [this]() { return get_agv_position(); });
}

bool Client::get_agv_position()
{
    get_agv_position_task_.start(asio::chrono::milliseconds(200));
    return true;
}

// 设置获取小车位置的时间间隔，单位毫秒
void Client::set_agv_position_interval(int interval_ms)
{
    get_agv_position_task_.set_interval(asio::chrono::milliseconds(interval_ms));
}
/**
 * @brief 删除获取小车位置的定时器 api
 *
 * 调用该函数后，将不再获取小车位置，直到再次调用 get_agv_position(handler)
 */
void Client::cancel_get_agv_position()
{
    get_agv_position_task_.stop();
}

std::string Client::get_response(ERROR_CODE ec, const std::string& msg)
{
    json jdata;
    jdata["code"] = +ec;
    jdata["message"] = msg;
    return jdata.dump();
}


/**
 * @brief 获取点云 api
 *
 * 该函数会创建一个定时器，每隔100ms向服务器发送一次请求。
 * 停止获取点云，调用 cancel_get_point_cloud() 函数
 *
 * @param handler 请求完成后的回调函数
 */
bool Client::get_point_cloud(ResponseHandler handler)
{
    return process_request("GET_POINT_CLOUD", handler, [this]() {return get_point_cloud();});
}

/**
 * @brief 获取点云 api
 */
bool Client::get_point_cloud()
{
    get_point_cloud_task_.start(asio::chrono::milliseconds(200));
    return true;
}

/**
 * @brief 删除获取点云的定时器 api
 *
 * 调用该函数后，将不再获取点云，直到再次调用 get_point_cloud(handler)
 */
void Client::cancel_get_point_cloud()
{
    get_point_cloud_task_.stop();
}

// 获取定位质量(废弃) api
bool Client::get_localization_quality(ResponseHandler handler)
{
    return process_request("LOCALIZATION_QUALITY", handler, [this]() {return get_localization_quality();});
}
bool Client::get_localization_quality()
{
    return send_request("LOCALIZATION_QUALITY", "");
}


// 获取电池和电机状态 api
bool Client::get_mcu2pc(ResponseHandler handler)
{
    return process_request("GET_MCU2PC", handler, [this]() {return get_mcu2pc();});
}
bool Client::get_mcu2pc()
{
    return send_request("GET_MCU2PC", "");
}


// 获取系统信息: 版本信息，系统资源信息 api
bool Client::get_sysinfo(ResponseHandler handler)
{
    return process_request("GET_SYSINFO", handler, [this]() {return get_sysinfo();});
}
bool Client::get_sysinfo()
{
    return send_request("GET_SYSINFO", "");
}

/**
 * @brief 删除获取小车系统信息的定时器 api
 *
 * 调用该函数后，将不再获取小车系统信息，直到再次调用 get_sysinfo(handler)
 */
void Client::cancel_get_sysinfo()
{
    if (get_sysinfo_running_ && get_sysinfo_timer_) {
        get_sysinfo_timer_->cancel();
    } else {
        log_info("get_sysinfo_timer_ is null");
    }
}



// 处理请求帮助函数
bool Client::process_request(const std::string request_name, ResponseHandler handler, std::function<bool()> task_func)
{
    if (!handler) {
        log_error("handler is not callable");
        return false;
    }
    if (!register_handler(request_name, handler)) {
        log_error(request_name + " has no cmd");
        return false;
    }

    return task_func();
}



// 获取3D点云 api
bool Client::get_3dcamera_pointcloud(ResponseHandler handler)
{

    return process_request("GET_CAMERA_POINT_CLOUD", handler, [this]() {return get_3dcamera_pointcloud();});
}

bool Client::get_3dcamera_pointcloud()
{
    get_3dcamera_pointcloud_task_.start(asio::chrono::milliseconds(200));
    return true;
}

// 单次获取3d点云，方便调试
// 如果没有连接过服务器就需要传递请求参数: {"ip": "server_ip", "port": "server_port"}
bool Client::get_3dcamera_pointcloud_single(const std::string& args, ResponseHandler handler)
{
    std::string ip = current_server_ip_;
    std::string port = current_server_port_;
    if (ip.empty() || port.empty()) {
        json jargs = json::parse(args, nullptr, false);
        if (jargs.is_discarded() || !jargs.is_object()) {
            log_error("invalid argument, please check your input");
            return false;
        }
        if (!jargs.contains("ip") || !jargs["ip"].is_string() ||
            !jargs.contains("port") || !jargs["port"].is_string()) {
            log_error("invalid argument field, please check your input");
            return false;
        }
        std::string ip = jargs.value("ip", "");
        std::string port = jargs.value("port", "");
        if (ip.empty() || port.empty()) {
            log_error("ip or port is empty");
            return false;
        }
    }

    log_info("pointcloud_connected_=%d", !!pointcloud_connected_);
    return process_request("GET_CAMERA_POINT_CLOUD", handler, [this]() {
        return send_request_on_pointcloud_socket("GET_CAMERA_POINT_CLOUD", "-3d");
    });

}

/**
 * @brief 删除获取小车摄像头点云的定时器 api
 *
 * 调用该函数后，将不再获取小车摄像头点云，直到再次调用 get_3dcamera_pointcloud(handler)
 */
void Client::cancel_get_3dcamera_pointcloud()
{
    get_3dcamera_pointcloud_task_.stop();
}

void Client::do_read_pointcloud()
{
    log_debug("start do read 3d");
    // 读取直到遇到分隔符
    asio::async_read_until(pointcloud_socket_, pointcloud_response_, "\r\n\r\n",
        [this](const std::error_code& ec, std::size_t bytes_transferred) {
            if (!ec) {
                // 构造字符串从buffer的开始位置到结束位置
                std::string reply(
                    asio::buffers_begin(pointcloud_response_.data()),
                    asio::buffers_begin(pointcloud_response_.data()) + bytes_transferred
                );

                // 消耗掉已经处理过的数据
                pointcloud_response_.consume(bytes_transferred);

                // 去除结尾的分隔符
                reply = reply.substr(0, reply.size() - 4);  //4 is the length of "\r\n\r\n"
                log_debug("get 3d, reply.size=%ld", reply.size());

                // 处理响应
                {
                    std::lock_guard<std::mutex> lock(pointcloud_handler_mutex_);
                    size_t delim_pos = reply.find('|');
                    if (delim_pos != std::string::npos) {
                        auto uuid = reply.substr(0, delim_pos);
                        log_debug("pointcloud uuid=%s", uuid.c_str());

                        auto iter = pointcloud_uuid2handlers_.find(uuid);
                        bool is_binary = (reply[delim_pos + 1] == '2' ? true : false);
                        std::string response;

                        if (is_binary) {
                            response = reply.substr(delim_pos + 3); // 3 is the length of "|2|"
                        } else {
                            response = reply.substr(delim_pos + 3); // 3 is the length of "|0|"
                        }

                        if (iter != pointcloud_uuid2handlers_.end()) {
                            iter->second(response);
                            pointcloud_uuid2handlers_.erase(iter);
                        }
                    }
                }

                // 继续读取下一个响应
                do_read_pointcloud();
            } else if (ec == asio::error::operation_aborted) {
                log_info("pointcloud read operation aborted.");
                // 即使操作被取消，也需要检查缓冲区中是否还有数据需要处理
                if (pointcloud_response_.size() > 0) {
                    // 模拟一次成功的读取来处理剩余数据
                    pointcloud_response_.consume(pointcloud_response_.size());
                }
            } else {
                log_error("pointcloud read error: %s", ec.message().c_str());
                close_pointcloud_socket();

                // 尝试重连点云连接
                // 使用非阻塞的异步定时器来延迟重连
                reconnect_pointcloud_after_delay();
            }
        });
}

// 在专用连接上发送请求的函数
bool Client::send_request_on_pointcloud_socket(const std::string& request_name, const std::string& request, const std::string& strUuid)
{
    std::unique_lock<std::mutex> lock(pointcloud_connection_mutex_);

    if (!pointcloud_socket_.is_open() || !pointcloud_connected_) {
        log_error("pointcloud socket is not open");

        // 尝试重连点云连接
        if (current_server_ip_.size() && current_server_port_.size()) {
            lock.unlock();
            connect_pointcloud_socket(current_server_ip_, current_server_port_,
                [](bool success) {
                    log_info("pointcloud reconnect %s", success ? "succeeded" : "failed");
                });
        }
        return false;
    }

    // 生成UUID
    std::string uuid = strUuid;
    if (uuid.empty()) {
        uuid = generate_uuid_v4();
    }

    // 注册回调
    ResponseHandler& handler = requestname2handlers_[request_name];
    pointcloud_uuid_to_handler_(uuid, handler);
    log_debug("[%s]->[%s], pointcloud_uuid2handlers_.size=%ld", request_name.c_str(), uuid.c_str(), pointcloud_uuid2handlers_.size());

    // 创建请求包
    std::string packet = create_packet(request_name, uuid, request);
    if (packet.empty()) {
        return false;
    }

    // 发送请求
    auto msg = std::make_shared<std::string>(packet);
    asio::async_write(pointcloud_socket_, asio::buffer(*msg),
        [this, msg](std::error_code ec, std::size_t bytes_transferred) {
        if (ec) {
            log_error("pointcloud write error: %s, value=%d", ec.message().c_str(), ec.value());

            // 点云连接失败时尝试重连
            if (!disconnect_by_user_) {
                close_pointcloud_socket();
                // 使用非阻塞的异步定时器来延迟重连
                reconnect_pointcloud_after_delay();
            }
        }
    });

    return true;
}

// 获取错误信息 api
bool Client::get_errors(ResponseHandler handler)
{
    return process_request("GET_ERRORS", handler, [this]() {return get_errors();});
}
bool Client::get_errors()
{
    return send_request("GET_ERRORS", "");
}


// 清除错误信息 api (直到底层再次上报错误信息)
bool Client::clear_errors(ResponseHandler handler)
{
    return process_request("CLEAR_ERRORS", handler, [this]() {return clear_errors();});
}
bool Client::clear_errors()
{
    return send_request("CLEAR_ERRORS", "");
}


bool Client::get_qr_camera_data(ResponseHandler handler)
{
    return process_request("GET_QR_CAMERA_DATA", handler, [this]() {return get_qr_camera_data();});
}

bool Client::get_qr_camera_data()
{
    get_qr_camera_data_task_.start(asio::chrono::milliseconds(200));
    return true;
}

/**
 * @brief 删除获取小车扫码相机的定时器 api
 *
 * 调用该函数后，将不再获取扫描相机数据，直到再次调用 get_qr_camera_data(handler)
 */
void Client::cancel_get_qr_camera_data()
{
    get_qr_camera_data_task_.stop();
}

// 避障有关

/**
 * @brief 获取小车避障点云的 api
 *
 * 客户请求该api后，api会创建一个定时器，定时器每隔 70ms 会向服务器发送一次请求.
 * 如果想停止获取避障点云数据，调用 cancel_get_scan2pointcloud() 函数
 * 想再次获取小车避障点云，再调用一次 get_scan2pointcloud([handler]) 函数
 *
 * @param handler 请求完成后的回调函数
 */
bool Client::get_scan2pointcloud(ResponseHandler handler)
{
    return process_request("GET_SCAN2POINTCLOUD", handler, [this]() {return get_scan2pointcloud();});
}

bool Client::get_scan2pointcloud()
{
    get_scan2pointcloud_task_.start(asio::chrono::milliseconds(200));
    return true;
}

/**
 * @brief 删除获取小车避障点云的定时器 api
 *
 * 调用该函数后，将不再获取避障点云，直到再次调用 get_scan2pointcloud(handler)
 */
void Client::cancel_get_scan2pointcloud()
{
    get_scan2pointcloud_task_.stop();
}

/**
 * @brief 获取小车避障轮廓的 api
 *
 * 客户请求该api后，api会创建一个定时器，定时器每隔 70ms 会向服务器发送一次请求.
 * 如果想停止获取避障轮廓数据，调用 cancel_get_obst_polygon() 函数
 * 想再次获取小车避障轮廓，再调用一次 get_scan2pointcloud([handler]) 函数
 *
 * @param handler 请求完成后的回调函数
 */
bool Client::get_obst_polygon(ResponseHandler handler)
{
    return process_request("GET_OBST_POLYGON", handler, [this]() {return get_obst_polygon();});
}

bool Client::get_obst_polygon()
{
    get_obst_polygon_task_.start(asio::chrono::milliseconds(200));
    return true;
}

/**
 * @brief 删除获取小车避障轮廓的定时器 api
 *
 * 调用该函数后，将不再获取避障轮廓，直到再次调用 get_obst_polygon(handler)
 */
void Client::cancel_get_obst_polygon()
{
    get_obst_polygon_task_.stop();
}

/**
 * @brief 获取小车障碍物点云的 api
 *
 * 客户请求该api后，api会创建一个定时器，定时器每隔 70ms 会向服务器发送一次请求.
 * 如果想停止获取障碍物点云，调用 cancel_get_obst_pcl() 函数
 * 想再次获取小车障碍物点云，再调用一次 get_obst_pcl([handler]) 函数
 *
 * @param handler 请求完成后的回调函数
 */
bool Client::get_obst_pcl(ResponseHandler handler)
{
    return process_request("GET_OBST_PCL", handler, [this]() {return get_obst_pcl();});
}

bool Client::get_obst_pcl()
{
    get_obst_pcl_task_.start(asio::chrono::milliseconds(200));
    return true;
}

/**
 * @brief 删除获取小车障碍物点云的定时器 api
 *
 * 调用该函数后，将不再获取障碍物点云，直到再次调用 get_obst_pcl(handler)
 */
void Client::cancel_get_obst_pcl()
{
    get_obst_pcl_task_.stop();
}

/**
 * @brief 获取小车模型轮廓的 api
 *
 * 客户请求该api后，api会创建一个定时器，定时器每隔 70ms 会向服务器发送一次请求.
 * 如果想停止获取模型轮廓数据，调用 cancel_get_model_polygon() 函数
 * 想再次获取小车模型轮廓，再调用一次 get_model_polygon([handler]) 函数
 *
 * @param handler 请求完成后的回调函数
 */
bool Client::get_model_polygon(ResponseHandler handler)
{
    return process_request("GET_MODEL_POLYGON", handler, [this]() {return get_model_polygon();});
}

bool Client::get_model_polygon()
{
    get_model_polygon_task_.start(asio::chrono::milliseconds(200));
    return true;
}

/**
 * @brief 删除获取小车模型轮廓的定时器 api
 *
 * 调用该函数后，将不再获取避障轮廓，直到再次调用 get_model_polygon(handler)
 */
void Client::cancel_get_model_polygon()
{
    get_model_polygon_task_.stop();
}

// 单次调用接口，不启动定时器
bool Client::get_scan2pointcloud_once(ResponseHandler handler) {
    return process_request("GET_SCAN2POINTCLOUD", handler, [this]() {
        return send_request("GET_SCAN2POINTCLOUD", "");
    });
}

bool Client::get_obst_polygon_once(ResponseHandler handler) {
    return process_request("GET_OBST_POLYGON", handler, [this]() {
        return send_request("GET_OBST_POLYGON", "");
    });
}

bool Client::get_obst_pcl_once(ResponseHandler handler) {
    return process_request("GET_OBST_PCL", handler, [this]() {
        return send_request("GET_OBST_PCL", "");
    });
}

bool Client::get_model_polygon_once(ResponseHandler handler) {
    return process_request("GET_MODEL_POLYGON", handler, [this]() {
        return send_request("GET_MODEL_POLYGON", "");
    });
}

}//end of namespace