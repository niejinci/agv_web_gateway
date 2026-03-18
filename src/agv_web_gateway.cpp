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

    // 5. 注册 HTTP 请求处理器
    m_server.set_http_handler(std::bind(&AgvWebGateway::on_http, this, std::placeholders::_1));
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

    // 开启 SO_REUSEADDR，解决重启时的 TIME_WAIT 端口占用问题
    m_server.set_reuse_addr(true);
    m_server.listen(asio::ip::tcp::v4(), ws_port);
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

        // 【新增】：处理获取地图列表请求
    if (cmd == "get_map_list") {
        // 调用底层的 API 获取地图列表
        m_client->get_map_list([this, hdl](const std::string& res) {
            // res 已经是底层返回的 JSON 字符串，我们需要给它包一层 "type":"map_list" 发给前端
            // 为了安全，直接拼接字符串
            std::string send_str = "{\"type\":\"map_list\",\"payload\":" + res + "}";

            // 将拼装好的 JSON 通过 WebSocket 发给前端
            m_server.send(hdl, send_str, websocketpp::frame::opcode::text);
        });
        return;
    }

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

// 实现 on_http 函数
void AgvWebGateway::on_http(websocketpp::connection_hdl hdl) {
    server::connection_ptr con = m_server.get_con_from_hdl(hdl);

    // 获取前端请求的路径 (例如 "/", "/style.css", "/app.js")
    std::string uri = con->get_resource();

    // 默认映射到 index.html
    std::string filename = "../web/index.html";
    std::string content_type = "text/html; charset=utf-8";

    if (uri == "/style.css") {
        filename = "../web/style.css";
        content_type = "text/css; charset=utf-8";
    } else if (uri == "/app.js") {
        filename = "../web/app.js";
        content_type = "application/javascript; charset=utf-8";
    }
    // 拦截所有对第三方库的请求，实现离线化支持
    else if (uri.find("/libs/") == 0) {
        // 直接拼接出 ../web/libs/xxx.js 的路径
        filename = "../web" + uri;
        content_type = "application/javascript; charset=utf-8";
    }
    //【全新替换】：不再写死 /SS27.pcd，而是拦截所有以 /pcd/ 开头的请求
    else if (uri.find("/pcd/") == 0) {
        // uri 的格式是 "/pcd/pc/SS27"
        std::string sub_path = uri.substr(5); // 截取出 "pc/SS27"

        size_t slash_pos = sub_path.find('/');
        if (slash_pos != std::string::npos) {
            std::string category = sub_path.substr(0, slash_pos);     // 取出 "pc"
            std::string map_name = sub_path.substr(slash_pos + 1);    // 取出 "SS27"

            // 【核心】：直接组装出 AGV 底层的绝对物理路径！
            // 结果如: /home/byd/data/map/pc/SS27/SS27.pcd
            std::string filepath = "/home/byd/data/map/" + category + "/" + map_name + "/" + map_name + ".pcd";

            // 严格的二进制内存块读取
            std::ifstream file(filepath, std::ios::in | std::ios::binary | std::ios::ate);
            if (file.is_open()) {
                std::streamsize size = file.tellg();
                file.seekg(0, std::ios::beg);
                std::string buffer;
                buffer.resize(size);

                if (file.read(&buffer[0], size)) {
                    con->set_body(buffer);
                    con->set_status(websocketpp::http::status_code::ok);
                    con->append_header("Content-Type", "application/octet-stream");
                    con->append_header("Connection", "close"); // 传完即关，防止拥堵
                    return; // 成功直接返回
                }
            }
            // 文件不存在或打开失败
            con->set_status(websocketpp::http::status_code::not_found);
            con->set_body("404 Not Found: Cannot open " + filepath + " (可能该厂房未生成3D点云地图)");
            return;
        }
    } else if (uri != "/") {
        // 请求了不存在的文件
        con->set_status(websocketpp::http::status_code::not_found);
        con->set_body("404 Not Found");
        return;
    }


    // 【修改开始：使用严格的二进制读取方式】
    // 注意：这里的 ios::ate 会在打开时把指针放到文件末尾，方便我们获取文件大小
    std::ifstream file(filename, std::ios::in | std::ios::binary | std::ios::ate);

    if (file.is_open()) {
        // 1. 获取文件的真实字节大小
        std::streamsize size = file.tellg();
        file.seekg(0, std::ios::beg); // 把指针移回文件开头

        // 2. 预先分配好内存空间，绝不会因为特殊字符截断
        std::string buffer;
        buffer.resize(size);

        // 3. 一次性把二进制数据读入内存
        if (file.read(&buffer[0], size)) {
            con->set_body(buffer);
            con->set_status(websocketpp::http::status_code::ok);
            con->append_header("Content-Type", content_type);
            // 【新增这一行】：明确告诉浏览器，传完大文件后主动关闭 HTTP 流，防止被占用
            con->append_header("Connection", "close");
        } else {
            con->set_status(websocketpp::http::status_code::internal_server_error);
            con->set_body("Error: Failed to read the binary file.");
        }
    } else {
        // 文件找不到时的报错
        con->set_status(websocketpp::http::status_code::not_found);
        con->set_body("404 Not Found: Cannot open file " + filename);
    }
}