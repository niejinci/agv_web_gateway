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

// 【新增】：自定义 WebSocket++ 底层配置
struct AgvGatewayConfig : public websocketpp::config::asio {
    // 将默认的 5000 毫秒 (5秒) 握手超时时间，大幅延长到 60000 毫秒 (60秒)
    // 这样网关就有充足的时间通过 AGV 的 Wi-Fi 把 2.5MB 的地图传给浏览器了
    static const long timeout_open_handshake = 60000;
};

// 【修改】：使用我们刚刚自定义的配置替换掉默认配置
// 注释掉旧的：typedef websocketpp::server<websocketpp::config::asio> server;
typedef websocketpp::server<AgvGatewayConfig> server;

// 定义我们使用的 websocket server 类型 (普通 TCP，无 SSL)
// typedef websocketpp::server<websocketpp::config::asio> server;

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
    void on_http(websocketpp::connection_hdl hdl);

    // 辅助发包函数，用于把原始 json 包装上类型后发给前端
    void send_to_frontend(websocketpp::connection_hdl hdl, const std::string& type, const std::string& raw_json);

    server m_server;
    asio::io_context m_io_context;
    std::shared_ptr<qclcpp::Client> m_client;   // 通信库实例
};