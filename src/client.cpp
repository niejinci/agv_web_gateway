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
    tmp_suffix_ = ".tmp";
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
    cancel_get_robot_state();

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

// 实现切换服务器的函数
void Client::switch_server(const std::string& new_host, const std::string& new_port, SwitchServerCallback callback)
{
    connect(new_host, new_port, callback);
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
                                clear_get_file_status();
                                clear_push_file_status();

                            } else {
                                // 成功发送后，立即尝试启动下一个写操作
                                // 这会形成一个自我驱动的“写循环”
                                start_next_write();
                            }
                        });
}

void Client::do_write_chunk()
{
    log_debug("chunk_number_=%d current input position=%d", chunk_number_, ifile_stream_upload_.tellg());
    if (ifile_stream_upload_.eof()) {
        // 完成所有块的发送，接收不一定完成
        ifile_stream_upload_.close();
        log_info("finish sending file");
        return;
    }

    std::string request;
    if (chunk_number_ == 0) {
        // 第一个块需要发送文件名
        std::string filename = get_filename_from_path(file_path_);
        // 开始标志|filename|filetype|fileSize
        std::string start_flag{"StartUploadFile"};
        request = start_flag;
        request += "|";
        request += filename;
        request += "|";
        request += file_type_;
        request += "|";
        request += std::to_string(file_size_);
        log_debug("request=%s", request.c_str());
    } else {
        ifile_stream_upload_.read(buffer_.data(), buffer_.size());
        auto bytes_read = ifile_stream_upload_.gcount();
        request.assign(buffer_.data(), bytes_read);
    }

    // 创建请求包
    std::string packet = create_packet("UPLOAD_FILE", upload_file_uuid_, request);
    if (packet.empty()) {
        ifile_stream_upload_.close();
        return;
    }

    // 发送数据
    asio::async_write(socket_, asio::buffer(packet),
                      [this](const asio::error_code& ec, std::size_t /*bytes_transferred*/) {
                          if (!ec) {
                            log_debug("finish sending chunk %d", chunk_number_);
                            ++chunk_number_;
                            // 成功发送后继续发送下一个块, 间隔 7ms
                            upload_file_timer_->expires_after(asio::chrono::milliseconds(7));
                            upload_file_timer_->async_wait(std::bind(&Client::start_upload_file, this, std::placeholders::_1));
                          } else {
                            clear_upload_file_status();
                            upload_handler_(get_response(ERROR_CODE::UPLOAD_FILE_FAILED , "async_write failed: " + ec.message()));
                          }
                      });
}

void Client::clear_upload_file_status()
{
    upload_file_failed_ = false;
    if (ifile_stream_upload_.is_open()) {
        ifile_stream_upload_.close();
    }

    {
        std::lock_guard<std::mutex> lock(handler_mutex_);
        uuid2handlers_.erase(upload_file_uuid_);
    }
}

void Client::start_upload_file(const asio::error_code& ec)
{
    if (!ec) {
        if (upload_file_failed_) {
            clear_upload_file_status();
            return;
        }
        if (!ifile_stream_upload_.is_open()) {
            return;
        }
        // 如果正在发送命令，等待一秒再尝试上传文件
        if (write_in_progress_) {
            upload_file_timer_->expires_after(asio::chrono::seconds(1));
            upload_file_timer_->async_wait(std::bind(&Client::start_upload_file, this, std::placeholders::_1));
            return;
        }

        do_write_chunk();
    } else {
        upload_handler_(get_response(ERROR_CODE::UPLOAD_FILE_FAILED , "File upload failed: " + ec.message()));
        ifile_stream_upload_.close();
    }
}

std::string Client::get_filename_from_path(const std::string& filepath)
{
    // 查找最后一个斜杠的位置
    size_t last_slash_pos = filepath.find_last_of("/\\");
    if (last_slash_pos == std::string::npos) {
        // 如果没有斜杠，整个路径就是文件名
        return filepath;
    }
    // 返回最后一个斜杠之后的部分
    return filepath.substr(last_slash_pos + 1);
}

/**
 * @brief 上传文件到服务器 api
 *
 * @param args JSON字符串，必须包含"filepath"字段，可选"type"字段
 *             格式示例: {"filepath": "/path/to/file", "type": "optional_type"}
 * @param handler 回调函数，用于处理上传完成后的响应
 * @return bool 返回布尔值表示上传初始化是否成功，不代表上传完成状态
 *
 * @note 该 api 是通用上传接口，适合小文件上传。
 */
bool Client::upload_file(const std::string& args, ResponseHandler handler)
{
    json jargs = json::parse(args, nullptr, false);
    if (jargs.is_discarded() || !jargs.is_object()) {
        log_error("invalid argument: " + args);
        return false;
    }
    if (!jargs.contains("filepath") || !jargs["filepath"].is_string()
            || (jargs.contains("type") && !jargs["type"].is_string())) {
        log_error("invalid argument field: " + args);
        return false;
    }
    std::string file_path = jargs["filepath"].get<std::string>();
    std::string type = jargs.value("type", "");
    return process_request("UPLOAD_FILE", handler, [this, &file_path, &type, handler]() {
        if (!socket_.is_open()) {
            log_error("socket is not open, please connect to server first");
            return false;
        }

        if (file_path.empty()) {
            log_error("file_path is empty");
            return false;
        }

        std::lock_guard<std::mutex> lck(upload_file_mutex_);
        if (ifile_stream_upload_.is_open()) {
            log_error(file_path_ + " is uploading, please wait");
            return false;
        }
        file_path_ = file_path;
        file_type_ = type;
        log_debug("file_path=%s", file_path.c_str());
        ifile_stream_upload_.open(file_path_, std::ios::binary);
        if (!ifile_stream_upload_) {
            log_error(file_path_ + " open failed");
            return false;
        }

        ifile_stream_upload_.seekg(0, std::ios::end);
        file_size_ = ifile_stream_upload_.tellg();
        ifile_stream_upload_.seekg(0, std::ios::beg);
        log_debug("file_size_=%d", file_size_);
        if (file_size_ <= 0) {
            ifile_stream_upload_.close();
            log_error(file_path_ + " is empty");
            return false;
        }
        chunk_number_ = 0;

        upload_file_uuid_ = generate_uuid_v4();
        uuid_to_handler_(upload_file_uuid_, handler);
        log_debug("upload_file_uuid_=%s", upload_file_uuid_.c_str());

        upload_handler_ = handler;
        upload_file_timer_ = std::make_shared<asio::steady_timer>(io_context_, asio::chrono::milliseconds(10));
        upload_file_timer_->async_wait(std::bind(&Client::start_upload_file, this, std::placeholders::_1));
        return true;
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
                        clear_upload_file_status();
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

void Client::get_file_callback(const std::string& reply)
{
    // 解析响应数据
    json obj = json::parse(reply, nullptr, false);
    if (obj.is_discarded() || !obj.is_object() || obj.value("code", -1) != 0) {
        get_file_finish_handler_(reply);
        clear_get_file_status();
        return;
    }

    if (obj.value("message", "") == "start") {
        // 服务器返回文件开始标志
        log_info("start get file: %s", output_file_name_.c_str());
        // 创建文件 && 清空文件内容
        ofile_stream_.open(output_file_name_, std::ios_base::out | std::ios_base::binary | std::ios_base::trunc);
        if (!ofile_stream_.is_open()) {
            get_file_finish_handler_(get_response(ERROR_CODE::OPEN_FILE_FAIL , output_file_name_ + " failed to open filed"));
            clear_get_file_status();
            return;
        }

        log_info("continue get file, file size=%ld", obj.value("total_size", 0));
        // 继续发送下一个请求
        send_request(get_file_request_name_, R"({"file_name": "", "hint": "continue"})", get_file_uuid_);
        return;
    }

    // 获取文件数据
    /*
        std::string 并不区分文本和二进制数据，但 nlohmann::json 库在序列化字符串时，会检查其是否为有效的 UTF-8 编码。
        当二进制数据中包含非 UTF-8 字节时，json::dump() 会抛出 type_error 异常：
        [json.exception.type_error.316] invalid UTF-8 byte at index 1: 0x01
    */
    // std::string data = obj.value("data", "");
    std::vector<uint8_t> vdata = obj["data"].get<std::vector<uint8_t>>();
    log_info("vdata.size=%lu", vdata.size());
    ofile_stream_.write(reinterpret_cast<const char*>(vdata.data()), vdata.size());

    if (obj.value("message", "") == "end") {
        // 服务器返回文件结束标志
        clear_get_file_status();
        {
            std::lock_guard<std::mutex> lock(handler_mutex_);
            uuid2handlers_.erase(get_file_uuid_);
        }
        std::string result_name = output_file_name_.substr(0, output_file_name_.size() - tmp_suffix_.size());
        json jdata;
        jdata["code"] = 0;
        jdata["message"] = "success";
        jdata["data"] = json();
        jdata["data"]["filename"] = result_name;
        //rename() 之前先删除结果文件，否则 rename() 会失败，报错:File exists
        std::remove(result_name.c_str());
        if (std::rename(output_file_name_.c_str(), result_name.c_str()) != 0) {
            jdata["code"] = +ERROR_CODE::RENAME_FILE_FAILED;
            jdata["message"] = strerror(errno);
            std::cerr << strerror(errno) << "\n";
        } else {
            log_info("get %s success", output_file_name_.c_str());
        }
        get_file_finish_handler_(jdata.dump());
    } else {
        // 继续发送下一个请求
        send_request(get_file_request_name_, R"({"file_name": "", "hint": "continue"})", get_file_uuid_);
    }
}

void Client::clear_get_file_status()
{
    get_file_in_progress_ = false;
    if (ofile_stream_.is_open()) {
        ofile_stream_.close();
    }
}

bool Client::is_directory(const std::string& path)
{
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
        return false;
    }
    return (info.st_mode & S_IFDIR);
}

/**
 * @brief 创建目录
 *
 * @param[in] file_name  [in] 文件命令，例如: pc/x/x.smap
 * @param[out] dir_path  [out] 要创建的目录名称，例如: map/pc/x/
 * @param[in] file_type  [in] 文件类型
 * @return true 创建成功
 * @return false 创建失败
 */
bool Client::create_directory(const std::string& file_name, std::string& dir_path, FILE_TYPE file_type)
{
    // 根据文件类型设置目录前缀
    dir_path = "map/";
    if (file_type == LOG_FILE) {
        dir_path = "log/";
    } else if (file_type == MODEL_FILE) {
        dir_path = "model/";
    } else if (file_type == VIDEO_FILE) {
        dir_path = "video/";
    } else if (file_type == TEACHIN_FILE) {
        dir_path = "teachin/";
    } else if (file_type == SHOWMAP_FILE) {
        dir_path = "showmap/";
    }

    // 解析文件名称中的相对路径
    // 查找文件名称中最后一个 /
    std::size_t pos = file_name.find_last_of('/');
    if (pos != std::string::npos) {
        // 输入中包含 /
        dir_path += file_name.substr(0, pos);
        dir_path += "/";
    }

    // 检查目录是否已存在
    if (is_directory(dir_path)) {
        return true;
    }

    // 循环查找路径中的每一个 '/'，逐级截取并创建目录
    // 例如 dir_path 为 "log/subdir/"，第一次循环处理 "log"，第二次处理 "log/subdir"
    std::size_t start = 0;
    std::size_t end;
    while ((end = dir_path.find('/', start)) != std::string::npos) {
        std::string subdir = dir_path.substr(0, end);
        log_info("mkdir subdir=%s", subdir.c_str());
        #ifdef _WIN32
        if (mkdir(subdir.c_str()) && errno != EEXIST) {
    #else
        if (mkdir(subdir.c_str(), 0777) && errno != EEXIST) {
    #endif
            return false;
        }

        // 更新搜索起点位置，继续寻找下一个 '/'
        start = end + 1;
    }

    return true;
}

/**
 * @brief 拉取地图 api
 *
 * 通过文件名拉取地图，先发送要获取的文件名称和开始标志，然后接收服务端的响应，
 * 如果服务端返回的是开始标志，那么就开始接收文件数据，如果服务端返回的是结束标志，那么就结束接收文件数据，
 * 除此之外，那么就继续接收文件数据
 *
 * @param file_name 要获取的地图名称
 * @param handler 完整获取地图后的回调函数
 * @return true 表示发起异步操作成功，实际结果通过传递给回调函数的参数判断
 * @return false 表示发起异步操作失败
 */
bool Client::pull_map(const std::string& file_name, ResponseHandler handler)
{
    auto result = is_ready_to_get_file(file_name, handler, MAP_FILE);
    if (!result) {
        log_error(result.error_message);
        return false;
    }
    return get_file_by_type(file_name, handler, MAP_FILE);
}

std::string Client::get_response(ERROR_CODE ec, const std::string& msg)
{
    json jdata;
    jdata["code"] = +ec;
    jdata["message"] = msg;
    return jdata.dump();
}

Client::OperationReult Client::is_ready_to_get_file(const std::string& file_name, ResponseHandler handler, FILE_TYPE type)
{
    if (!handler) {
        return OperationReult::fail("handler is empty");
    }

    if (file_name.empty() || std::all_of(file_name.begin(), file_name.end(), [](int c){ return std::isspace(c); })) {
        return OperationReult::fail("get file name is empty");
    }

    std::lock_guard<std::mutex> lck(get_file_mutex_);
    if (get_file_in_progress_) {
        return OperationReult::fail(output_file_name_ + " is getting, please wait");
    }

    std::string dir_path;
    if (!create_directory(file_name, dir_path, type)) {
        return OperationReult::fail("create directory failed: " + dir_path);
    }

    get_file_request_name_ = "PULL_MAP";
    if (type == LOG_FILE) {
        get_file_request_name_ = "GET_LOG_FILE";
    } else if (type == MODEL_FILE) {
        get_file_request_name_ = "GET_MODEL_FILE";
    } else if (type == VIDEO_FILE) {
        get_file_request_name_ = "GET_CAMERA_VIDEO";
    } else if (type == TEACHIN_FILE) {
        get_file_request_name_ = "GET_TEACHIN_FILE";
    }
    if (!register_handler(get_file_request_name_, std::bind(&Client::get_file_callback, this, std::placeholders::_1))) {
        return OperationReult::fail(get_file_request_name_ + " has no cmd");
    }

    return OperationReult::ok();
}

bool Client::get_file_by_type(const std::string& file_name, ResponseHandler handler, FILE_TYPE type)
{
    output_file_name_ = "./map/";
    if (type == LOG_FILE) {
        output_file_name_ = "./log/";
    } else if (type == MODEL_FILE) {
        output_file_name_ = "./model/";
    } else if (type == VIDEO_FILE) {
        output_file_name_ = "./video/";
    } else if (type == TEACHIN_FILE) {
        output_file_name_ = "./teachin/";
    }
    output_file_name_ += file_name;
    //获取的文件先保存到临时文件，在成功获取完成后再 rename 过去
    output_file_name_ += tmp_suffix_;
    get_file_finish_handler_ = handler;
    get_file_uuid_ = generate_uuid_v4();
    log_debug("output_file_name_=%s, get_file_uuid_+%s", output_file_name_.c_str(), get_file_uuid_.c_str());

    auto ret = send_request(get_file_request_name_, R"({"file_name": ")" + file_name + R"(", "hint": "start"})", get_file_uuid_);
    if (ret) {
        get_file_in_progress_ = true;
    } else {
        clear_get_file_status();
    }
    return ret;
}

// 推送地图
void Client::push_file_callback(const std::string& reply)
{
    // 解析响应数据
    json obj = json::parse(reply, nullptr, false);
    if (obj.is_discarded() || !obj.is_object() || obj.value("code", -1) != 0) {
        push_file_finish_handler_(reply);
        clear_push_file_status();
        {
            std::lock_guard<std::mutex> lock(handler_mutex_);
            uuid2handlers_.erase(push_file_uuid_);
        }
        return;
    }

    json jdata;
    if (obj.value("message", "") == "continue") {
        log_debug("\ncontinue push file");

        // 开始分块读取文件并发送
        if (!ifile_stream_.eof()) {
            ifile_stream_.read(push_buffer_.data(), push_buffer_.size());
            size_t count = ifile_stream_.gcount();

            current_push_size_ += count;
            jdata["hint"] = "push";
            if (ifile_stream_.eof()) {
                jdata["hint"] = "end";
            }

            if (count == 0 && current_push_size_ < push_file_size_) {
                jdata["message"] = "read_file_failed";
                jdata["code"] = +ERROR_CODE::READ_FILE_FAIL;
                json jtmp;
                jtmp["total_size"] = push_file_size_;
                jtmp["current_push_size"] = current_push_size_;
                jdata["data"] = jtmp;
                // 告诉服务端上传文件出错了
                jdata["hint"] = "error";
                send_request("PUSH_MAP", jdata.dump(), push_file_uuid_);
                return;
            }

            jdata["data"] = std::vector<uint8_t>(push_buffer_.data(), push_buffer_.data() + count);
            jdata["total_size"] = push_file_size_;
            jdata["current_push_size"] = current_push_size_;

            // 继续发送下一个请求
            send_request("PUSH_MAP", jdata.dump(), push_file_uuid_);
        }
    } else {
        json jtmp;
        jtmp["total_size"] = push_file_size_;
        jtmp["current_push_size"] = current_push_size_;
        jdata["data"] = jtmp;
        if (push_file_size_ > current_push_size_) {
            jdata["code"] = +ERROR_CODE::PUSH_FILE_FAIL;
            jdata["message"] = "push_file_failed";
        } else {
            jdata["code"] = 0;
            jdata["message"] = "success";
        }
        push_file_finish_handler_(jdata.dump());
        clear_push_file_status();
        {
            std::lock_guard<std::mutex> lock(handler_mutex_);
            uuid2handlers_.erase(push_file_uuid_);
        }
    }
}

void Client::clear_push_file_status()
{
    push_file_in_progress_ = false;
    current_push_size_ = 0;
    if (ifile_stream_.is_open()) {
        ifile_stream_.close();
    }
}

/**
 * @brief 推送地图
 *    先发送要推送的文件名称和开始标志，然后接收服务端的响应，
 *    如果服务端返回的是continue标志，那么就继续发送文件块
 *    发送最后一个块时，带上end标志
 * @param file_path 要推送的地图名称
 * @param handler 完成推送地图后的回调函数
 * @return true 表示发起异步操作成功，实际结果通过传递给回调函数的参数判断
 * @return false 表示发起异步操作失败
 */
bool Client::push_map(const std::string& file_path, ResponseHandler handler)
{
    if (!handler) {
        log_error("handler is empty");
        return false;
    }
    if (file_path.empty()) {
        log_error("file_path is empty");
        return false;
    }

    if (!register_handler("PUSH_MAP", std::bind(&Client::push_file_callback, this, std::placeholders::_1))) {
        log_error("PUSH_MAP has no cmd");
        return false;
    }

    std::lock_guard<std::mutex> lck(push_file_mutex_);
    if (push_file_in_progress_) {
        log_error(file_path + " is pushing, please wait");
        return false;
    }

    push_file_in_progress_ = true;
    push_file_finish_handler_ = handler;
    push_file_uuid_ = generate_uuid_v4();
    std::string file_name = get_filename_from_path(file_path);
    log_debug("file_path=%s, push_file_uuid_=%s file_name=%s", file_path.c_str(), push_file_uuid_.c_str(), file_name.c_str());

    ifile_stream_.open(file_path, std::ios_base::binary);
    if (!ifile_stream_.is_open()) {
        log_error(file_path + " open failed");
        clear_push_file_status();
        return false;
    }
    ifile_stream_.seekg(0, std::ios::end);
    push_file_size_ = ifile_stream_.tellg();
    ifile_stream_.seekg(0, std::ios::beg);
    log_debug("push_file_size_=%u", push_file_size_);

    return send_request("PUSH_MAP", R"({"file_name": ")" + file_name + R"(", "hint": "start"})", push_file_uuid_);
}

/**
 * @brief 设置小车控制模式 api
 *
 * @param mode 见头文件中 "操作模式常量" 定义
 */
bool Client::set_operating_mode(int mode, ResponseHandler handler)
{
    return process_request("SET_OPERATING_MODE", handler, [this, mode]() {return set_operating_mode(mode);});
}

bool Client::set_operating_mode(int mode)
{
    json jdata;
    jdata["mode"] = mode;
    return send_request("SET_OPERATING_MODE", jdata.dump());
}

// 获取操作模式 api
bool Client::get_operating_mode(ResponseHandler handler)
{
    return process_request("GET_OPERATING_MODE", handler, [this]() {return get_operating_mode();});
}
/**
 * @brief 获取小车控制模式
 *
 * 小车控制模式由客户端设置，本地会维护设置的模式，客户端重启的时候通过该接口获取小车当前的控制模式
 * (跟奉工沟通，这个控制模式只能由qt客户端设置，rcs不能设置)
 * @param mode
 */
bool Client::get_operating_mode()
{
    return send_request("GET_OPERATING_MODE", "");
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

// 获取地图文件列表 api
bool Client::get_map_list(ResponseHandler handler)
{
    return process_request("GET_MAP_LIST", handler, [this]() {return get_map_list();});
}

bool Client::get_map_list()
{
    return send_request("GET_MAP_LIST", "");
}

// 获取日志文件列表 api
bool Client::get_log_list(ResponseHandler handler)
{
    return process_request("GET_LOG_LIST", handler, [this]() {return get_log_list();});
}

bool Client::get_log_list()
{
    return send_request("GET_LOG_LIST", "");
}

// 重启或关闭主机 api
bool Client::reboot_or_poweroff(const std::string& command, ResponseHandler handler)
{
    return process_request("REBOOT_OR_POWEROFF", handler, [this, &command]() {
        std::string cmd;
        json jargs = json::parse(command, nullptr, false);
        if (jargs.is_discarded() || !jargs.is_object()) {
            // 如果解析失败，那么认为传入的参数是原始的字符串，例如: reboot, poweroff
            cmd = command;
        } else {
            cmd = jargs.value("command", "");
        }

        return send_request("REBOOT_OR_POWEROFF", R"({"command": ")" + cmd + R"("})");
    });
}

// 获取日志文件 api
bool Client::get_log_file(const std::string& file_name, ResponseHandler handler)
{
    auto result = is_ready_to_get_file(file_name, handler, LOG_FILE);
    if (!result) {
        log_error(result.error_message);
        return false;
    }
    return get_file_by_type(file_name, handler, LOG_FILE);
}

// 获取模型文件 api
bool Client::get_model_file(const std::string& file_name, ResponseHandler handler)
{
    std::string file_name_tmp{file_name};
    if (file_name_tmp.empty()) {
        // 默认文件名
        file_name_tmp = "agv_model_para.json";
    }
    auto result = is_ready_to_get_file(file_name_tmp, handler, MODEL_FILE);
    if (!result) {
        log_error(result.error_message);
        return false;
    }
    return get_file_by_type(file_name_tmp, handler, MODEL_FILE);
}

/**
 * @brief 检测服务器 ip 可连接性 api
 *
 * @param ip_list
 * @param handler
 */
void Client::check_connectivity(const std::vector<std::string>& ip_list, ResponseHandler handler)
{
    auto results = std::make_shared<std::vector<std::pair<std::string, bool>>>();
    auto remaining = std::make_shared<std::atomic<int>>(ip_list.size());
    log_info("start check connectivity at %lu", std::time(nullptr));
    for (const auto& ip : ip_list) {
        auto socket = std::make_shared<asio::ip::tcp::socket>(io_context_);
        auto timer = std::make_shared<asio::steady_timer>(io_context_);
        connect_with_timeout(ip, socket, timer, results, remaining, handler);
    }
}

void Client::connect_with_timeout(const std::string& ip,
                                    std::shared_ptr<asio::ip::tcp::socket> socket,
                                    std::shared_ptr<asio::steady_timer> timer,
                                    std::shared_ptr<std::vector<std::pair<std::string, bool>>> results,
                                    std::shared_ptr<std::atomic<int>> remaining, ResponseHandler handler)
{
    asio::ip::tcp::resolver resolver(io_context_);
    auto endpoints = resolver.resolve(ip, "9034");

    timer->expires_after(std::chrono::seconds(3));
    timer->async_wait([socket](const asio::error_code& ec) {
        if (!ec) {
            socket->cancel();
        }
    });

    asio::async_connect(*socket, endpoints, [ip, socket, timer, results, remaining, handler](const std::error_code& ec, const asio::ip::tcp::endpoint&) {
        timer->cancel();
        results->emplace_back(ip, !ec);
        if (--(*remaining) == 0) {
            log_info("finish check connectivity at %lu", std::time(nullptr));
            json result_json = json::array();
            for (const auto& result : *results) {
                result_json.push_back({{"ip", result.first}, {"connected", result.second}});
            }
            handler(result_json.dump());
        }
    });
}

// 设置系统时间 api
bool Client::set_datetime(const std::string& args, ResponseHandler handler)
{
    return process_request("SET_DATE_TIME", handler, [this, &args]() {
        return send_request("SET_DATE_TIME", R"({"datetime": ")" + args + R"("})");
    });
}

// 获取系统时间 api
bool Client::get_datetime(ResponseHandler handler)
{
    return process_request("GET_DATE_TIME", handler, [this]() {return get_datetime();});
}

bool Client::get_datetime()
{
    return send_request("GET_DATE_TIME", R"({"command": "date +\"%Y-%m-%d %H:%M:%S\""})");
}

/**
 * @brief 执行终端命令 api
 *
 * @param command
        {
            "command": "",
            "tty": "1bbe1a48-2ee1-4e05-8b16-cfcd7e9edc30",
            "next_chunk": 2
        }
 */
bool Client::terminal_command(const std::string& args, ResponseHandler handler)
{
    return process_request("TERMINAL_COMMAND", handler, [this, &args]() {
        json jargs = json::parse(args, nullptr, false);
        if (jargs.is_discarded() || !jargs.is_object()) {
            log_error("parameter format is invalid");
            return false;
        }
        if ((jargs.contains("tty") && !jargs["tty"].is_string()) ||
            (jargs.contains("next_chunk") && !jargs["next_chunk"].is_number())) {
                log_error("invalid parameter: tty or next_chunk");
            return false;
        }

        std::string uuid = jargs.value("tty", "");
        return send_request("TERMINAL_COMMAND", args, uuid);
    });
}

// 建图 api
bool Client::build_map(ResponseHandler handler)
{
    return process_request("BUILD_MAPPING", handler, [this]() {return build_map();});
}
bool Client::build_map()
{
    return send_request("BUILD_MAPPING", "");
}

/**
 * @brief OTA升级 api
 *
 * 该函数用于发起或管理设备的OTA（空中）升级流程。它接收JSON格式的参数，
 * 支持"upgrade"（开始升级）和"disupgrade"（取消升级）两种命令。
 * 对于升级命令，需要指定目标版本。函数会设置一个定时器来轮询升级进度，
 * 并通过回调函数返回升级状态。
 *
 * @param args JSON格式的参数字符串，格式为{"command": "upgrade|disupgrade", "version": "版本号"}
 * @param handler 回调函数，用于处理升级过程中的响应
 * @return 如果请求发送成功返回true，参数验证失败或发送失败返回false
 */
bool Client::ota_upgrade(const std::string& args, ResponseHandler handler)
{
    if (!handler) {
        log_error("handler is not callable");
        return false;
    }
    if (!ota_upgrade_timer_) {
        log_debug("ota_upgrade_timer_ is null");
        ota_upgrade_timer_ = std::make_shared<asio::steady_timer>(io_context_);
    }
    json jargs = json::parse(args, nullptr, false);
    if (jargs.is_discarded() || !jargs.is_object()) {
        log_error("parameter format is invalid");
        return false;
    }
    if (!jargs.contains("command") || !jargs["command"].is_string() ||
        (jargs.contains("version") && !jargs["version"].is_string())) {
            log_error("invalid parameter: command or version");
            return false;
    }
    std::string command = jargs.value("command", "");
    if (command != "upgrade" && command != "disupgrade") {
        log_error("unsupported command");
        return false;
    }
    std::string version = jargs.value("version", "");
    if (command == "upgrade" && version.empty()) {
        log_error("version is empty");
        return false;
    }
    log_info("command=%s version=%s", command.c_str(), version.c_str());

    if (!register_handler("OTA_UPGRADE", [this, command, version, handler](const std::string& reply) {
        log_info("command=%s version=%s", command.c_str(), version.c_str());
        if (command == "disupgrade") {
            handler(reply);
            return;
        }

        /*
            "code": 0,
            "message": "",
            "data":
                "percentage": 15,
                "msg": "unzip fail"
            }
        */
        json jreply = json::parse(reply, nullptr, false);
        if (jreply.is_discarded() || !jreply.is_object()) {
            log_warn("reply format is invalid");
            return;
        }

        // 升级失败
        int code = jreply.value("code", 0);
        if (code) {
            handler(reply);
            return;
        }

        std::string msg = jreply["data"].value("msg", "success");
        int percentage = jreply["data"].value("percentage", 0);
        log_info("percentage=%d, msg=%s", percentage, msg.c_str());
        handler(reply);
        if (percentage < 100) {
            // 升级中
            ota_upgrade_timer_->expires_after(asio::chrono::seconds(5));
            ota_upgrade_timer_->async_wait([this, version](const asio::error_code& ec) {
                if (!ec) {
                    send_request("OTA_UPGRADE", R"({"command": "status", "version":")" + version + R"("})");
                }
            });
        }
    })) {
        log_error("OTA_UPGRADE has no cmd");
        return false;
    }

    log_info("start ota upgrade");
    return send_request("OTA_UPGRADE", args);
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

// 获取速度 api
bool Client::get_velocity(ResponseHandler handler)
{
    return process_request("GET_VELOCITY", handler, [this]() {return get_velocity();});
}
bool Client::get_velocity()
{
    return send_request("GET_VELOCITY", "");
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

// 获取当前运行任务 api
bool Client::get_run_task(ResponseHandler handler)
{
    return process_request("GET_RUN_TASK", handler, [this]() {return get_run_task();});
}
bool Client::get_run_task()
{
    return send_request("GET_RUN_TASK", "");
}

// 重定位操作 api
bool Client::relocation(const std::string& args, ResponseHandler handler)
{
    return process_request("RELOCATION", handler, [this, &args]() {return relocation(args);});
}
bool Client::relocation(const std::string& args)
{
    return send_request("RELOCATION", args);
}

// 平动操作 api
bool Client::translation(const std::string& args, ResponseHandler handler)
{
    return process_request("TRANSLATION", handler, [this, &args]() {return translation(args);});
}
bool Client::translation(const std::string& args)
{
    return send_request("TRANSLATION", args);
}

// 旋转操作 api
bool Client::rotation(const std::string& args, ResponseHandler handler)
{
    return process_request("ROTATION", handler, [this, &args]() {return rotation(args);});
}
bool Client::rotation(const std::string& args)
{
    return send_request("ROTATION", args);
}

// 顶升操作 api
bool Client::lifting(const std::string& args, ResponseHandler handler)
{
    return process_request("LIFTING", handler, [this, &args]() {return lifting(args);});
}
bool Client::lifting(const std::string& args)
{
    return send_request("LIFTING", args);
}

// 遥控操作 api
bool Client::remote_control(const std::string& args, ResponseHandler handler)
{
    return process_request("REMOTE_CONTROL", handler, [this, &args]() {return remote_control(args);});
}
bool Client::remote_control(const std::string& args)
{
    return send_request("REMOTE_CONTROL", args);
}

// 启动任务 api
bool Client::start_task(const std::string& args, ResponseHandler handler)
{
    return process_request("START_TASK", handler, [this, &args]() {return start_task(args);});
}
bool Client::start_task(const std::string& args)
{
    return send_request("START_TASK", args);
}

// 设置日志级别 api 方便调试用
bool Client::set_log_level(const std::string& args, ResponseHandler handler)
{
    return process_request("SET_LOG_LEVEL", handler, [this, &args]() {return set_log_level(args);});
}
bool Client::set_log_level(const std::string& args)
{
    return send_request("SET_LOG_LEVEL", args);
}

// 取消任务 api
bool Client::cancel_task(const std::string& args, ResponseHandler handler)
{
    return process_request("CANCEL_TASK", handler, [this, &args]() {return cancel_task(args);});
}
bool Client::cancel_task(const std::string& args)
{
    return send_request("CANCEL_TASK", args);
}

// 恢复任务 api
bool Client::resume_task(ResponseHandler handler)
{
    return process_request("RESUME_TASK", handler, [this]() {return resume_task();});
}
bool Client::resume_task()
{
    return send_request("RESUME_TASK", "");
}

// 暂停任务 api
bool Client::pause_task(ResponseHandler handler)
{
    return process_request("PAUSE_TASK", handler, [this]() {return pause_task();});
}
bool Client::pause_task()
{
    return send_request("PAUSE_TASK", "");
}

// 获取连接到服务器的客户端信息 api 增加可调式性
bool Client::get_clients(ResponseHandler handler)
{
    return process_request("GET_CLIENTS", handler, [this]() {return get_clients();});
}
bool Client::get_clients()
{
    return send_request("GET_CLIENTS", "");
}

// 急停操作 api
bool Client::emergency_stop(const std::string& args, ResponseHandler handler)
{
    return process_request("EMERGENCY_STOP", handler, [this, &args]() {return emergency_stop(args);});
}
bool Client::emergency_stop(const std::string& args)
{
    return send_request("EMERGENCY_STOP", args);
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

/**
 * @brief 获取系统信息: 版本信息，系统资源信息 api
 *
 * 客户请求该api后，api会创建一个定时器，定时器每隔1s会向服务器发送一次请求.
 * 如果想在停止获取系统信息时，调用 cancel_get_sysinfo() 函数
 * 想再次获取小车系统信息时，再调用一次 get_sysinfo([handler]) 函数
 *
 * @param handler 请求完成后的回调函数
 */
bool Client::get_sysinfo_periodic(ResponseHandler handler)
{
    return process_request("GET_SYSINFO", handler, [this]() {
        if (!get_sysinfo_timer_) {
            log_info("get_sysinfo_timer_ is null, make it");
            get_sysinfo_timer_ = std::make_shared<asio::steady_timer>(io_context_);
        }

        if (!get_sysinfo_running_) {
            log_info("start get_sysinfo periodic");
            get_sysinfo_periodic(asio::error_code());
        } else {
            log_info("get_sysinfo is already running");
        }
        return true;
    });
}

void Client::get_sysinfo_periodic(const asio::error_code& ec)
{
    // 如果定时器被取消，那么不再需要定期获取系统信息了
    if (ec == asio::error::operation_aborted) {
        log_info("agv timer canceled, %s", ec.message().c_str());
        get_sysinfo_running_ = false;
        return;
    }
    get_sysinfo_running_ = true;
    send_request("GET_SYSINFO", "");
    get_sysinfo_timer_->expires_after(asio::chrono::seconds(1));
    get_sysinfo_timer_->async_wait([this](const asio::error_code& ec) {
        get_sysinfo_periodic(ec);
    });
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

// 托盘旋转 api
bool Client::pallet_rotation(const std::string& args, ResponseHandler handler)
{
    return process_request("PALLET_ROTATION", handler, [this, &args]() {return pallet_rotation(args);});
}
bool Client::pallet_rotation(const std::string& args)
{
    return send_request("PALLET_ROTATION", args);
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

// 获取摄像头视频文件列表 api
bool Client::get_camera_video_list(ResponseHandler handler)
{
    return process_request("GET_CAMERA_VIDEO_LIST", handler, [this]() {return get_camera_video_list();});
}

bool Client::get_camera_video_list()
{
    return send_request("GET_CAMERA_VIDEO_LIST", "");
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

// 取消充电 api
bool Client::stop_charging(ResponseHandler handler)
{
    return process_request("STOP_CHARGING", handler, [this]() {return stop_charging();});
}
bool Client::stop_charging()
{
    return send_request("STOP_CHARGING", "");
}

// 设置输出 api
bool Client::set_do(const std::string& args, ResponseHandler handler)
{
    return process_request("SET_DO", handler, [this, &args]() {return set_do(args);});
}
bool Client::set_do(const std::string& args)
{
    return send_request("SET_DO", args);
}

// 设置输入 api
// 小车等待数字量输入变化
bool Client::set_di(const std::string& args, ResponseHandler handler)
{
    return process_request("WAIT_DI", handler, [this, &args]() {return set_di(args);});
}
bool Client::set_di(const std::string& args)
{
    return send_request("WAIT_DI", args);
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

// 获取进程信息 api
bool Client::get_processes_info(const std::string& args, ResponseHandler handler)
{
    return process_request("GET_PROCESSES_INFO", handler, [this, &args]() {
        return send_request("GET_PROCESSES_INFO", args);
    });
}

// 检查 socket 的 TCP keepalive 设置 api
bool Client::check_fd_keep_alive(const std::string& fd, ResponseHandler handler)
{
    return process_request("CHECK_FD_KEEP_ALIVE", handler, [this, &fd]() {
        return send_request("CHECK_FD_KEEP_ALIVE", fd);
    });
}

bool Client::get_plc_digital_io(const std::string& args, ResponseHandler handler)
{
    return process_request("GET_PLC_DIGITAL_IO", handler, [this, &args]() {
        return send_request("GET_PLC_DIGITAL_IO", args);
    });
}

bool Client::get_wifi_list(ResponseHandler handler)
{
    return process_request("GET_WIFI_LIST", handler, [this]() {
        return send_request("GET_WIFI_LIST", "");
    });
}

bool Client::set_wifi_config(const std::string& args, ResponseHandler handler)
{
    return process_request("SET_WIFI_CONFIG", handler, [this, &args]() {
        return send_request("SET_WIFI_CONFIG", args);
    });
}

bool Client::get_network_interface(ResponseHandler handler)
{
    return process_request("GET_NETWORK_INTERFACE", handler, [this]() {
        return send_request("GET_NETWORK_INTERFACE", "");
    });
}

bool Client::get_execution_queue(ResponseHandler handler)
{
    return process_request("GET_EXECUTION_QUEUE", handler, [this]() {
        return send_request("GET_EXECUTION_QUEUE", "");
    });
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

// 双臂机器人控制 API 的实现
// 使能机器人
bool Client::enable_robot(const std::string& args, ResponseHandler handler)
{
    return process_request("ENABLE_ROBOT", handler, [this, &args]() { return enable_robot(args); });
}
bool Client::enable_robot(const std::string& args)
{
    return send_request("ENABLE_ROBOT", args);
}

// 切换坐标系
bool Client::set_coordinate_system(const std::string& args, ResponseHandler handler)
{
    return process_request("SET_COORDINATE_SYSTEM", handler, [this, &args]() { return set_coordinate_system(args); });
}
bool Client::set_coordinate_system(const std::string& args)
{
    return send_request("SET_COORDINATE_SYSTEM", args);
}

// 切换工具
bool Client::set_tool(const std::string& args, ResponseHandler handler)
{
    return process_request("SET_TOOL", handler, [this, &args]() { return set_tool(args); });
}
bool Client::set_tool(const std::string& args)
{
    return send_request("SET_TOOL", args);
}

// 5. 单轴点动控制
bool Client::jog_single_axis(const std::string& args, ResponseHandler handler)
{
    return process_request("JOG_SINGLE_AXIS", handler, [this, &args]() { return jog_single_axis(args); });
}
bool Client::jog_single_axis(const std::string& args)
{
    return send_request("JOG_SINGLE_AXIS", args);
}

bool Client::one_click_homing(const std::string& args, ResponseHandler handler)
{
    return process_request("ONE_CLICK_HOMING", handler, [this, &args]() { return one_click_homing(args); });
}
bool Client::one_click_homing(const std::string& args)
{
    return send_request("ONE_CLICK_HOMING", args);
}

// 获取示教点位数据文件列表 api
bool Client::get_teachin_file_list(ResponseHandler handler)
{
    return process_request("GET_TEACHIN_FILE_LIST", handler, [this]() {return get_teachin_file_list();});
}

bool Client::get_teachin_file_list()
{
    return send_request("GET_TEACHIN_FILE_LIST", "");
}

// 获取示教点位数据文件 api
bool Client::get_teachin_file(const std::string& file_name, ResponseHandler handler)
{
    auto result = is_ready_to_get_file(file_name, handler, TEACHIN_FILE);
    if (!result) {
        log_error(result.error_message);
        return false;
    }
    return get_file_by_type(file_name, handler, TEACHIN_FILE);
}

// 推送示教点位数据到服务端保存 api
bool Client::push_teachin_points(const std::string& points, ResponseHandler handler)
{
    return process_request("PUSH_TEACHIN_POINTS", handler, [this, &points]() {return push_teachin_points(points);});
}

bool Client::push_teachin_points(const std::string& points)
{
    return send_request("PUSH_TEACHIN_POINTS", points);
}

// 获取示教点位数据文件内容 api
bool Client::get_teachin_points(const std::string& file_name, ResponseHandler handler)
{
    return process_request("GET_TEACHIN_POINTS", handler, [this, &file_name]() {return get_teachin_points(file_name);});
}

bool Client::get_teachin_points(const std::string& file_name)
{
    return send_request("GET_TEACHIN_POINTS", file_name);
}

bool Client::get_robot_state(ResponseHandler handler)
{
    return process_request("GET_ROBOT_STATE", handler, [this]() {return get_robot_state();});
}

bool Client::get_robot_state()
{
    // 20hz
    get_robot_state_task_.start(asio::chrono::milliseconds(50));
    return true;
}

/**
 * @brief 删除获取机器人状态的定时器 api
 *
 * 调用该函数后，将不再获取机器人状态，直到再次调用 get_robot_state(handler)
 */
void Client::cancel_get_robot_state()
{
    get_robot_state_task_.stop();
}

// 获取底盘信息api
bool Client::get_chassis_info(ResponseHandler handler)
{
    return process_request("GET_CHASSIS_INFO", handler, [this]() {return get_chassis_info();});
}

bool Client::get_chassis_info()
{
    return send_request("GET_CHASSIS_INFO", "");
}

// 删除示教点位数据文件 api
bool Client::delete_teachin_files(const std::string& filenames, ResponseHandler handler)
{
    return process_request("DELETE_TEACHIN_FILES", handler, [this, &filenames]() {return delete_teachin_files(filenames);});
}
bool Client::delete_teachin_files(const std::string& filenames)
{
    return send_request("DELETE_TEACHIN_FILES", filenames);
}

// 获取消息代理(RCS)连接状态 api
bool Client::get_broker_connection(ResponseHandler handler)
{
    return process_request("GET_BROKER_CONNECTION", handler, [this]() {return get_broker_connection();});
}

bool Client::get_broker_connection()
{
    return send_request("GET_BROKER_CONNECTION", "");
}

// 控制 rcs 上下线
bool Client::set_rcs_online(const std::string& args, ResponseHandler handler)
{
    return process_request("SET_RCS_ONLINE", handler, [this, &args]() { return set_rcs_online(args); });
}

bool Client::set_rcs_online(const std::string& args)
{
    return send_request("SET_RCS_ONLINE", args);
}

// 软复位 api
bool Client::soft_reset(ResponseHandler handler)
{
    return process_request("SOFT_RESET", handler, [this]() { return soft_reset(); });
}

bool Client::soft_reset()
{
    return send_request("SOFT_RESET", "");
}

// GET_RACK_NUMBER
bool Client::get_rack_number(ResponseHandler handler)
{
    return process_request("GET_RACK_NUMBER", handler, [this]() {return get_rack_number();});
}

bool Client::get_rack_number()
{
    return send_request("GET_RACK_NUMBER", "");
}

// 检查显示用的地图更新状态 api
bool Client::check_showmap_update_status(const std::string& args, ResponseHandler handler)
{
    return process_request("CHECK_SHOWMAP_UPDATE_STATUS", handler, [this, &args]() {return check_showmap_update_status(args);});
}

bool Client::check_showmap_update_status(const std::string& args)
{
    return send_request("CHECK_SHOWMAP_UPDATE_STATUS", args);
}

// 开始建图
bool Client::start_mapping(ResponseHandler handler)
{
    return process_request("START_MAPPING", handler, [this]() {return start_mapping();});
}
bool Client::start_mapping()
{
    return send_request("START_MAPPING", "");
}

// 保存定位用地图
bool Client::save_location_map(const std::string& args, ResponseHandler handler)
{
    return process_request("SAVE_LOCATION_MAP", handler, [this, &args]() {return save_location_map(args);});
}

bool Client::save_location_map(const std::string& args)
{
    return send_request("SAVE_LOCATION_MAP", args);
}

// 结束建图
bool Client::end_mapping(ResponseHandler handler)
{
    return process_request("END_MAPPING", handler, [this]() {return end_mapping();});
}

bool Client::end_mapping()
{
    return send_request("END_MAPPING", "");
}

// 货架管理 API 的实现
// 获取货架列表 api
bool Client::get_wares(ResponseHandler handler)
{
    return process_request("GET_WARES", handler, [this]() {return get_wares();});
}
bool Client::get_wares()
{
    return send_request("GET_WARES", "");
}

// 删除货架条目 api
bool Client::delete_ware(const std::string& args, ResponseHandler handler)
{
    return process_request("DELETE_WARE", handler, [this, &args]() {return delete_ware(args);});
}
bool Client::delete_ware(const std::string& args)
{
    return send_request("DELETE_WARE", args);
}

// 修改货架条目 api
bool Client::modify_ware(const std::string& args, ResponseHandler handler)
{
    return process_request("MODIFY_WARE", handler, [this, &args]() {return modify_ware(args);});
}
bool Client::modify_ware(const std::string& args)
{
    return send_request("MODIFY_WARE", args);
}

// 添加货架条目 api
bool Client::add_ware(const std::string& args, ResponseHandler handler)
{
    return process_request("ADD_WARE", handler, [this, &args]() {return add_ware(args);});
}
bool Client::add_ware(const std::string& args)
{
    return send_request("ADD_WARE", args);
}

// map_data 是包含了 .pgm, .yaml, .json 三个文件的压缩包，服务端会解压并把三个文件放在一起
bool Client::upload_map_data(const std::string& file_path, ResponseHandler handler)
{
    std::string formatted_file_path = file_path;
    // 去掉 formatted_file_path 首尾空白
    formatted_file_path.erase(0, formatted_file_path.find_first_not_of(" \t\n\r\f\v"));
    formatted_file_path.erase(formatted_file_path.find_last_not_of(" \t\n\r\f\v") + 1);
    return upload_file(R"({"filepath":")" + formatted_file_path + R"(", "type": "map"})", handler);
}

// map_name 是地图文件的前缀，服务端会根据这个前缀找到对应的 .pgm, .yaml, .json 三个文件，并把三个文件打包成一个压缩包提供下载
bool Client::download_map_data(const std::string& map_name, ResponseHandler handler)
{
    // 为了提高客户端使用体验，当输入的 map_name 为:
    // 2d-1
    // 2d-1.zip
    // pc/2d-1/2d-1
    // pc/2d-1/2d-1.zip
    // 这几种情况时，统一把 map_name 处理成 pc/2d-1/2d-1.zip 的格式，这样用户输入更灵活一些
    // 注意，本地编译环境只支持到 c++14
    std::string formatted_map_name = map_name;
    // 去掉 formatted_map_name 首尾空白
    formatted_map_name.erase(0, formatted_map_name.find_first_not_of(" \t\n\r\f\v"));
    formatted_map_name.erase(formatted_map_name.find_last_not_of(" \t\n\r\f\v") + 1);

    if (!formatted_map_name.empty()) {
        // 去掉 .zip 后缀
        if (formatted_map_name.size() > 4 && formatted_map_name.substr(formatted_map_name.size() - 4) == ".zip") {
            formatted_map_name = formatted_map_name.substr(0, formatted_map_name.size() - 4);
        }
        // 如果没有路径，则添加默认路径前缀
        if (formatted_map_name.find('/') == std::string::npos) {
            formatted_map_name = "pc/" + formatted_map_name + "/" + formatted_map_name;
        }
        // 添加 .zip 后缀
        formatted_map_name += ".zip";
    } else {
        log_error("map_name is empty");
        return false;
    }

    auto result = is_ready_to_get_file(formatted_map_name, handler, MAP_FILE);
    if (!result) {
        log_error(result.error_message);
        return false;
    }
    return get_file_by_type(formatted_map_name, handler, MAP_FILE);
}

}//end of namespace