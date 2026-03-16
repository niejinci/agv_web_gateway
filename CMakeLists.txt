cmake_minimum_required(VERSION 3.10)
project(agv_web_gateway)

# 设置 C++ 标准为 C++14 或以上 (你的 client 库可能需要)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 告诉 websocketpp 我们使用的是独立的 asio，不依赖 boost
add_definitions(-DASIO_STANDALONE)
add_definitions(-D_WEBSOCKETPP_CPP11_STL_)

# 添加头文件搜索路径
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/asio/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/websocketpp
)

# 查找系统线程库 (asio 和 websocketpp 需要)
find_package(Threads REQUIRED)

# 添加所有的源文件
set(SOURCES
    src/main.cpp
    src/agv_web_gateway.cpp
    src/client.cpp   # 这是你的通信库源码，一起编译
)

add_executable(${PROJECT_NAME} ${SOURCES})

# 链接线程库
target_link_libraries(${PROJECT_NAME} Threads::Threads)