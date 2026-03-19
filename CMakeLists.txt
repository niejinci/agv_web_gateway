cmake_minimum_required(VERSION 3.10)
project(agv_web_gateway)

# 设置 C++ 标准为 C++14 或以上 (你的 client 库可能需要)
set(CMAKE_CXX_STANDARD 17)
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
find_package(fmt REQUIRED)

# 添加所有的源文件
set(SOURCES
    src/main.cpp
    src/agv_web_gateway.cpp
    src/client.cpp   # 这是你的通信库源码，一起编译
)

add_executable(${PROJECT_NAME} ${SOURCES})

# 链接线程库
target_link_libraries(${PROJECT_NAME} Threads::Threads fmt::fmt)

# ===================================================================
# ==== 在 cmake 阶段直接拷贝依赖资源 (不需要 make install) ====
# ==== 资源拷贝与路径管理 (支持脚本指定路径 & 本地默认兜底) ====
# ===================================================================

# 1. 针对可执行文件和 web 目录的兜底
if(NOT DEFINED EXECUTABLE_OUTPUT_PATH)
    # 如果没传宏，默认把二进制放在 build/bin 下
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/bin)
    # 显式告诉 CMake 把可执行文件输出到兜底目录，而不是默认的 build/ 根目录
    set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
endif()

# 此时 EXECUTABLE_OUTPUT_PATH 绝对有值了，立刻拷贝 web 文件夹到它上一级
# 例如: build/bin 的上一级就是 build/web，正好符合程序里 "../web" 的预期
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/web
     DESTINATION ${EXECUTABLE_OUTPUT_PATH}/..)
message(STATUS "Web directory mapped to: ${EXECUTABLE_OUTPUT_PATH}/../web")

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config
     DESTINATION ${EXECUTABLE_OUTPUT_PATH}/..)
message(STATUS "Config directory mapped to: ${EXECUTABLE_OUTPUT_PATH}/../config")
