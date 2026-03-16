#pragma once

// 必须在引入 websocketpp 之前定义 ASIO_STANDALONE (也可以在 CMake 中定义)
#ifndef ASIO_STANDALONE
#define ASIO_STANDALONE
#endif

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <memory>
#include <string>

// 引入你的通信库头文件
#include "client.h"

// 定义我们使用的 websocket server 类型 (普通 TCP，无 SSL)
typedef websocketpp::server<websocketpp::config::asio> server;

class AgvWebGateway {
public:
    AgvWebGateway();
    ~AgvWebGateway();

    // 启动网关，连接 AGV 服务端，并开启 WebSocket 监听
    void run(const std::string& agv_ip, uint16_t agv_port, uint16_t ws_port);

private:
    // WebSocket 回调函数
    void on_open(websocketpp::connection_hdl hdl);
    void on_close(websocketpp::connection_hdl hdl);
    void on_message(websocketpp::connection_hdl hdl, server::message_ptr msg);

    // 辅助发包函数，用于把原始 json 包装上类型后发给前端
    void send_to_frontend(websocketpp::connection_hdl hdl, const std::string& type, const std::string& raw_json);

    server m_server;
    asio::io_context m_io_context;
    std::shared_ptr<qclcpp::Client> m_client;   // 通信库实例
};