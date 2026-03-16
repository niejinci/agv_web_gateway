#include "agv_web_gateway.h"
#include <iostream>

int main(int argc, char* argv[]) {
    try {
        AgvWebGateway gateway;
        // 连接到你的 agv server (假设是本机 9034)，对外提供 8080 WebSocket 服务
        gateway.run("127.0.0.1", 9034, 8080);
    } catch (const std::exception& e) {
        std::cerr << "网关发生异常: " << e.what() << std::endl;
    }
    return 0;
}