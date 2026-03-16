#include "agv_web_gateway.h"
#include <iostream>
#include <functional>

AgvWebGateway::AgvWebGateway() {
    // 1. 初始化 WebSocket 日志 (为了控制台清爽，我们关闭一些冗余日志)
    m_server.clear_access_channels(websocketpp::log::alevel::all);
    m_server.set_access_channels(websocketpp::log::alevel::connect);
    m_server.set_access_channels(websocketpp::log::alevel::disconnect);
    m_server.set_access_channels(websocketpp::log::alevel::app);

    // 2. 初始化 ASIO
    // 【核心魔法】把我们自己的 io_context 传给 websocketpp！
    // 这样 websocketpp 就不会自己创建了，而是和我们的 client 用同一个。
    m_server.init_asio(&m_io_context);
    // m_server.init_asio(); // 这是默认行为，会自己创建一个 io_context，我们就无法在外部访问了

    // 3. 注册 WebSocket 回调
    m_server.set_open_handler(std::bind(&AgvWebGateway::on_open, this, std::placeholders::_1));
    m_server.set_close_handler(std::bind(&AgvWebGateway::on_close, this, std::placeholders::_1));
    m_server.set_message_handler(std::bind(&AgvWebGateway::on_message, this, std::placeholders::_1, std::placeholders::_2));

    // 4. 实例化你的通信库
    m_client = qclcpp::Client::create(m_io_context, "../config/requestname2cmd.ini");
}

AgvWebGateway::~AgvWebGateway() {
    m_server.stop();
}

void AgvWebGateway::run(const std::string& agv_ip, uint16_t agv_port, uint16_t ws_port) {
    std::cout << "[Gateway] 连接 AGV 服务端 " << agv_ip << ":" << agv_port << " ..." << std::endl;
    // 假设你的 client 有类似 connect 的方法，如果没有请根据实际修改
    m_client->connect(agv_ip, std::to_string(agv_port), [agv_port](bool success) {
        if (success) {
            std::cout << "[Gateway] 成功连接到 AGV 服务端! port=" << agv_port << std::endl;
        } else {
            std::cerr << "[Gateway] 连接 AGV 服务端失败! port=" << agv_port << std::endl;
        }
    });

    std::cout << "[Gateway] WebSocket 服务已启动，监听端口: " << ws_port << std::endl;
    m_server.listen(ws_port);
    m_server.start_accept();
    // websocketpp 的 run() 内部源码其实就是： m_io_context->run();
    // 这一句跑起来，WebSocket 的网络事件和 Client 的网络事件就都在这个主线程里被分发处理了！
    m_server.run(); // 阻塞运行，基于 asio 的事件循环
}

void AgvWebGateway::on_open(websocketpp::connection_hdl hdl) {
    std::cout << "[Gateway] 浏览器前端已连接!" << std::endl;
}

void AgvWebGateway::on_close(websocketpp::connection_hdl hdl) {
    std::cout << "[Gateway] 浏览器前端断开连接." << std::endl;
    // 可以在这里调用 cancel 系列 API，停止给下位机发请求
    // m_client->cancel_get_point_cloud();
}

void AgvWebGateway::on_message(websocketpp::connection_hdl hdl, server::message_ptr msg) {
    std::string cmd = msg->get_payload();
    std::cout << "[Gateway] 收到前端指令: " << cmd << std::endl;

    // 以下点云，服务端支持的最高请求频率 5hz。
    // 小车点云(稀疏，显示用)
    if (cmd == "start_point_cloud") {
        m_client->get_point_cloud([this, hdl](const std::string& json_resp) {
            send_to_frontend(hdl, "point_cloud", json_resp);
        });
    } else if (cmd == "stop_point_cloud") {
        m_client->cancel_get_point_cloud();
    }
    // 获取小车位置
    else if (cmd == "start_agv_position") {
        m_client->get_agv_position([this, hdl](const std::string& json_resp) {
            send_to_frontend(hdl, "agv_position", json_resp);
        });
    } else if (cmd == "stop_agv_position") {
        m_client->cancel_get_agv_position();
    }
    // 获取小车避障轮廓
    else if (cmd == "start_obst_polygon") {
        m_client->get_obst_polygon([this, hdl](const std::string& json_resp) {
            send_to_frontend(hdl, "obst_polygon", json_resp);
        });
    } else if (cmd == "stop_obst_polygon") {
        m_client->cancel_get_obst_polygon();
    }
    // 获取小车避障点云(避障用)
    else if (cmd == "start_scan2pointcloud") {
        m_client->get_scan2pointcloud([this, hdl](const std::string& json_resp) {
            send_to_frontend(hdl, "scan2pointcloud", json_resp);
        });
    } else if (cmd == "stop_scan2pointcloud") {
        m_client->cancel_get_scan2pointcloud();
    }
    // 获取小车模型轮廓
    else if (cmd == "start_model_polygon") {
        m_client->get_model_polygon([this, hdl](const std::string& json_resp) {
            send_to_frontend(hdl, "model_polygon", json_resp);
        });
    } else if (cmd == "stop_model_polygon") {
        m_client->cancel_get_model_polygon();
    }
    // 获取小车障碍物点云
    else if (cmd == "start_obst_pcl") {
        m_client->get_obst_pcl([this, hdl](const std::string& json_resp) {
            send_to_frontend(hdl, "obst_pcl", json_resp);
        });
    } else if (cmd == "stop_obst_pcl") {
        m_client->cancel_get_obst_pcl();
    }
    // 获取日志文件列表，作为一个非持续数据流的测试接口
    else if (cmd == "get_log_list") {
        m_client->get_log_list([this, hdl](const std::string& json_resp) {
            send_to_frontend(hdl, "log_list", json_resp);
        });
    }  else {
        std::cout << "[Gateway] 未知指令: " << cmd << std::endl;
    }
    // TODO: 其他接口...
}

void AgvWebGateway::send_to_frontend(websocketpp::connection_hdl hdl, const std::string& type, const std::string& raw_json) {
    // 将你的 API 返回的纯 JSON，包装成 {"type": "...", "payload": {...}} 的格式
    // 这样前端才知道收到的是点云还是坐标
    std::string ws_msg = "{\"type\": \"" + type + "\", \"payload\": " + raw_json + "}";

    // websocketpp 的 send 是线程安全的，可以在你的通信库的异步回调线程中直接调用
    std::error_code ec;
    m_server.send(hdl, ws_msg, websocketpp::frame::opcode::text, ec);
    if (ec) {
        // 前端可能断开了，忽略或记录日志
    }
}