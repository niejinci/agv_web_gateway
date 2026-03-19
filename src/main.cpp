#include "agv_web_gateway.h"
#include <iostream>

int main(int argc, char* argv[])
{
    LogManager::getInstance().initialize("/home/byd/log/agv_web_gateway", "gateway");
    LogManager::getInstance().getLogger()->info("log instance initialize success");
    try {
        AgvWebGateway gateway;
        // 连接到你的 agv server (假设是本机 9034)，对外提供 9090 WebSocket 服务
        gateway.run("127.0.0.1", 9034, 9090);
    } catch (const std::exception& e) {
        // std::cerr << "网关发生异常: " << e.what() << std::endl;
        LogManager::getInstance().getLogger()->info("网关发生异常: {}", e.what());
    }
    return 0;
}