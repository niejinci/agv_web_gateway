#ifndef CLIENT_H
#define CLIENT_H

#include <asio.hpp>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <mutex>
#include <queue>
#include <fstream>
#include "periodic_task.h"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#define SOCKET int
#define INVALID_SOCKET (-1)
#define closesocket close
#endif

namespace qclcpp {
const std::string requestname2cmd_file = "D:/byd_agv_in_gitee/agv_qtclient/requestname2cmd.ini";

enum FILE_TYPE {
    MAP_FILE = 0,
    LOG_FILE = 1,
    MODEL_FILE = 2,
    VIDEO_FILE = 3,
    TEACHIN_FILE = 4,
    SHOWMAP_FILE = 5,
};

enum class ERROR_CODE {
    SUCCESS = 0,
    OPEN_FILE_FAIL,
    UPLOAD_FILE_FAILED,
    PUSH_FILE_FAIL,
    READ_FILE_FAIL,
    RENAME_FILE_FAILED,
    GET_FILE_FAILED,
};

constexpr auto operator+(ERROR_CODE a) noexcept
{
    return static_cast<std::underlying_type_t<ERROR_CODE>>(a);
}

class Client : public std::enable_shared_from_this<Client> {
public:
    using ResponseHandler = std::function<void(const std::string&)>;
    using SwitchServerCallback = std::function<void(bool)>;

    static std::shared_ptr<Client> create(asio::io_context& io_context, const std::string& requestname2cmd_file="");
    void connect(const std::string& host, const std::string& port, std::function<void(bool)> callback);
    bool upload_file(const std::string& args, ResponseHandler handler);

    void disconnect(std::function<void(const std::string& error)> callback = default_disconnect_callback);
    void switch_server(const std::string& new_host, const std::string& new_port, SwitchServerCallback callback = default_switch_callback);
    bool pull_map(const std::string& map_name, ResponseHandler handler);
    bool push_map(const std::string& file_path, ResponseHandler handler);
    bool get_agv_position(ResponseHandler handler);
    void set_agv_position_interval(int interval_ms);
    bool set_operating_mode(int mode, ResponseHandler handler);
    bool get_operating_mode(ResponseHandler handler);
    bool get_point_cloud(ResponseHandler handler);
    bool get_map_list(ResponseHandler handler);
    bool get_camera_video_list(ResponseHandler handler);

    bool get_log_list(ResponseHandler handler);
    bool get_log_file(const std::string& file_name, ResponseHandler handler);
    bool reboot_or_poweroff(const std::string& command, ResponseHandler handler);
    bool set_datetime(const std::string& datetime, ResponseHandler handler);
    bool get_datetime(ResponseHandler handler);

    bool terminal_command(const std::string& args, ResponseHandler handler);
    bool build_map(ResponseHandler handler);
    bool ota_upgrade(const std::string& args, ResponseHandler handler);
    bool get_localization_quality(ResponseHandler handler);
    bool get_velocity(ResponseHandler handler);
    bool get_mcu2pc(ResponseHandler handler);
    bool get_run_task(ResponseHandler handler);
    bool pause_task(ResponseHandler handler);
    bool resume_task(ResponseHandler handler);
    bool cancel_task(const std::string& args, ResponseHandler handler);
    bool start_task(const std::string& args, ResponseHandler handler);
    bool rotation(const std::string& args, ResponseHandler handler);
    bool translation(const std::string& args, ResponseHandler handler);
    bool relocation(const std::string& args, ResponseHandler handler);
    bool lifting(const std::string& args, ResponseHandler handler);
    bool remote_control(const std::string& args, ResponseHandler handler);
    bool set_log_level(const std::string& args, ResponseHandler handler);
    bool get_clients(ResponseHandler handler);
    bool emergency_stop(const std::string& args, ResponseHandler handler);
    bool get_sysinfo(ResponseHandler handler);
    void cancel_get_sysinfo();
    // 定时获取小车系统信息
    bool get_sysinfo_periodic(ResponseHandler handler);
    bool pallet_rotation(const std::string& args, ResponseHandler handler);
    bool get_model_file(const std::string& file_name, ResponseHandler handler);
    bool stop_charging(ResponseHandler handler);
    bool set_do(const std::string& args, ResponseHandler handler);
    bool set_di(const std::string& args, ResponseHandler handler);
    bool clear_errors(ResponseHandler handler);

    // 检测ip列表的连接性
    void check_connectivity(const std::vector<std::string>& ip_list, ResponseHandler handler);

    // 本地操作，无需命令字段
    void cancel_get_agv_position();
    void cancel_get_point_cloud();
    void stop_heart_beat();
    void restart_heart_beat();

    // 默认的切换服务器回调函数
    static void default_switch_callback(bool success);
    static void default_disconnect_callback(const std::string& error);

    bool get_3dcamera_pointcloud_single(const std::string& args, ResponseHandler handler);
    bool get_3dcamera_pointcloud(ResponseHandler handler);
    void cancel_get_3dcamera_pointcloud();
    bool get_errors(ResponseHandler handler);
    bool get_processes_info(const std::string& args, ResponseHandler handler);
    bool check_fd_keep_alive(const std::string& fd, ResponseHandler handler);

    bool get_plc_digital_io(const std::string& args, ResponseHandler handler);
    bool get_wifi_list(ResponseHandler handler);
    bool set_wifi_config(const std::string& args, ResponseHandler handler);
    bool get_network_interface(ResponseHandler handler);
    bool get_execution_queue(ResponseHandler handler);
    bool get_qr_camera_data(ResponseHandler handler);
    void cancel_get_qr_camera_data();

    // 避障有关
    bool get_scan2pointcloud(ResponseHandler handler);
    void cancel_get_scan2pointcloud();

    bool get_obst_polygon(ResponseHandler handler);
    void cancel_get_obst_polygon();

    bool get_obst_pcl(ResponseHandler handler);
    void cancel_get_obst_pcl();

    bool get_model_polygon(ResponseHandler handler);
    void cancel_get_model_polygon();

    bool get_chassis_info(ResponseHandler handler);

    // 新增：双臂机器人控制 API
    bool enable_robot(const std::string& args, ResponseHandler handler);
    bool set_coordinate_system(const std::string& args, ResponseHandler handler);
    bool set_tool(const std::string& args, ResponseHandler handler);
    bool jog_single_axis(const std::string& args, ResponseHandler handler);
    bool one_click_homing(const std::string& args, ResponseHandler handler);
    bool get_teachin_file_list(ResponseHandler handler);
    bool get_teachin_file(const std::string& file_name, ResponseHandler handler);
    bool push_teachin_points(const std::string& points, ResponseHandler handler);
    bool get_teachin_points(const std::string& file_name, ResponseHandler handler);
    bool delete_teachin_files(const std::string& filenames, ResponseHandler handler);

    bool get_robot_state(ResponseHandler handler);
    void cancel_get_robot_state();

    bool get_broker_connection(ResponseHandler handler);
    bool set_rcs_online(const std::string& args, ResponseHandler handler);
    bool soft_reset(ResponseHandler handler);
    bool get_rack_number(ResponseHandler handler);
    bool check_showmap_update_status(const std::string& args, ResponseHandler handler);
    bool start_mapping(ResponseHandler handler);
    bool save_location_map(const std::string& args, ResponseHandler handler);
    bool end_mapping(ResponseHandler handler);
    bool get_wares(ResponseHandler handler);
    bool delete_ware(const std::string& args, ResponseHandler handler);
    bool modify_ware(const std::string& args, ResponseHandler handler);
    bool add_ware(const std::string& args, ResponseHandler handler);

    bool upload_map_data(const std::string& file_path, ResponseHandler handler);
    bool download_map_data(const std::string& map_name, ResponseHandler handler);
public:
    // 单次调用接口
    bool get_scan2pointcloud_once(ResponseHandler handler);
    bool get_obst_polygon_once(ResponseHandler handler);
    bool get_obst_pcl_once(ResponseHandler handler);
    bool get_model_polygon_once(ResponseHandler handler);

private:
    Client(asio::io_context& io_context, const std::string& requestname2cmd_file_arg="");
    void do_read();
    static std::string generate_uuid_v4();
    void uuid_to_handler_(const std::string& uuid, ResponseHandler handler);
    void pointcloud_uuid_to_handler_(const std::string& uuid, ResponseHandler handler);
    std::string create_packet(const std::string& request_name, const std::string& uuid, const std::string& msg);
    void start_next_write();
    void do_write_chunk();
    void close_socket(std::function<void(const std::string& error)> callback = default_disconnect_callback);
    void clear_write_status();
    void reconnect_pointcloud_after_delay();

    // 使用 atomic 分别控制不同任务的状态，互不干扰
    std::atomic<bool> is_downloading_video_{false};
    std::atomic<bool> is_downloading_map_{false};

    struct DownloadResult {
        bool success;
        std::string error_message;
        operator bool() const { return success; }
    };

    struct OperationReult {
        bool success;
        std::string error_message;
        operator bool() const { return success;}

        // 方便的创建成功时候的结果
        static OperationReult ok(const std::string msg="") {
            return {true, msg};
        }

        // 方便的创建失败时候的结果
        static OperationReult fail(const std::string msg="") {
            return {false, msg};
        }
    };

    // 文件上传
    std::string file_type_;
    std::string upload_file_uuid_;
    int file_size_;
    int chunk_number_;
    std::string file_path_;
    void start_upload_file(const asio::error_code& ec);
    std::string get_filename_from_path(const std::string& filepath);
    std::shared_ptr<asio::steady_timer> upload_file_timer_;
    bool upload_file_failed_ = false;
    void clear_upload_file_status();

    static const uint16_t sync_field_ = 0x4E66;
    // 添加专用于点云数据的连接
    asio::ip::tcp::socket pointcloud_socket_;
    asio::streambuf pointcloud_response_;
    bool pointcloud_connected_{false};
    std::mutex pointcloud_connection_mutex_;
    void close_pointcloud_socket();

    bool main_connected_ = false;
    asio::io_context& io_context_;
    asio::ip::tcp::socket socket_;
    asio::streambuf response_;
    std::unordered_map<std::string, ResponseHandler> requestname2handlers_;
    std::unordered_map<std::string, ResponseHandler> uuid2handlers_;
    std::unordered_map<std::string, ResponseHandler> pointcloud_uuid2handlers_;
    std::ifstream ifile_stream_upload_;
    std::array<char, 8192> buffer_;
    ResponseHandler upload_handler_;
    std::mutex handler_mutex_;
    std::mutex pointcloud_handler_mutex_;
    std::queue<std::string> write_queue_;
    std::mutex write_queue_mutex_;
    std::mutex upload_file_mutex_;
    bool write_in_progress_ = false;
    bool load_file(const std::string& file_path);
    std::unordered_map<std::string, uint16_t> requestname2cmd_;

    // 推送地图
    uint32_t current_push_size_ = 0;
    uint32_t push_file_size_;
    std::array<char, 8192> push_buffer_;
    std::mutex push_file_mutex_;
    bool push_file_in_progress_ = false;
    std::string push_file_uuid_;
    std::ifstream ifile_stream_;
    ResponseHandler push_file_finish_handler_;
    void push_file_callback(const std::string& reply);
    void clear_push_file_status();

    // 拉取地图
    std::mutex get_file_mutex_;
    bool get_file_in_progress_ = false;
    std::string get_file_uuid_;
    std::ofstream ofile_stream_;
    ResponseHandler get_file_finish_handler_;
    std::string output_file_name_;
    std::string tmp_suffix_;
    void get_file_callback(const std::string& reply);
    void clear_get_file_status();
    bool create_directory(const std::string& file_name, std::string& dir_path, FILE_TYPE file_type=MAP_FILE);

    // ota 升级
    std::shared_ptr<asio::steady_timer> ota_upgrade_timer_;

    // 添加的新函数声明
    void connect_pointcloud_socket(const std::string& host, const std::string& port, std::function<void(bool)> callback);
    void do_read_pointcloud();
    bool send_request_on_pointcloud_socket(const std::string& request_name, const std::string& request, const std::string& strUuid = "");


    // 自动重连
    std::shared_ptr<asio::steady_timer> heartbeat_timer_;
    std::string current_server_ip_;
    std::string current_server_port_;
    void automatic_reconnect();
    void heart_beat_callback(const std::string& reply);
    void heart_beat(const asio::error_code& ec);
    bool disconnect_by_user_ = false;
    std::function<void(const std::string&, const std::string&)> connect_lambda;
    bool heart_beat_running_ = false;

    // 带超时的连接
    void connect_with_timeout(const std::string& ip,
        std::shared_ptr<asio::ip::tcp::socket> socket,
        std::shared_ptr<asio::steady_timer> timer,
        std::shared_ptr<std::vector<std::pair<std::string, bool>>> results,
        std::shared_ptr<std::atomic<int>> remaining, ResponseHandler handler);

    // 根据类型获取文件
    bool get_file_by_type(const std::string& map_name, ResponseHandler handler, FILE_TYPE type);
    OperationReult is_ready_to_get_file(const std::string& file_name, ResponseHandler handler, FILE_TYPE type=MAP_FILE);
    std::string get_file_request_name_;

    bool get_run_task();
    bool get_mcu2pc();
    bool get_velocity();
    bool get_localization_quality();
    bool build_map();
    bool get_datetime();
    bool get_log_list();
    bool get_map_list();
    bool get_camera_video_list();
    bool get_operating_mode();
    bool set_operating_mode(int mode);
    bool get_3dcamera_pointcloud();
    bool pause_task();
    bool resume_task();
    bool cancel_task(const std::string& args);
    bool start_task(const std::string& args);
    bool rotation(const std::string& args);
    bool pallet_rotation(const std::string& args);
    bool translation(const std::string& args);
    bool relocation(const std::string& args);
    bool lifting(const std::string& args);
    bool remote_control(const std::string& args);
    bool set_log_level(const std::string& args);
    bool get_clients();
    bool emergency_stop(const std::string& args);
    bool get_sysinfo();
    std::shared_ptr<asio::steady_timer> get_sysinfo_timer_;
    bool get_sysinfo_running_ = false;
    void get_sysinfo_periodic(const asio::error_code& ec);

    bool get_camera_point_cloud();
    bool get_errors();
    bool stop_charging();
    bool set_do(const std::string& args);
    bool set_di(const std::string& args);
    bool clear_errors();
    bool get_chassis_info();
    bool get_broker_connection();
    bool set_rcs_online(const std::string& args);
    bool soft_reset();
    bool get_rack_number();
    bool check_showmap_update_status(const std::string& args);
    bool start_mapping();
    bool save_location_map(const std::string& args);
    bool end_mapping();
    bool get_wares();
    bool delete_ware(const std::string& args);
    bool modify_ware(const std::string& args);
    bool add_ware(const std::string& args);

    // 新增：双臂机器人控制 API 的私有实现
    bool enable_robot(const std::string& args);
    bool set_coordinate_system(const std::string& args);
    bool set_tool(const std::string& args);
    bool jog_single_axis(const std::string& args);
    bool one_click_homing(const std::string& args);
    bool get_teachin_file_list();
    bool push_teachin_points(const std::string& points);
    bool get_teachin_points(const std::string& file_name);
    bool delete_teachin_files(const std::string& filenames);

    bool is_directory(const std::string& path);
    std::string get_response(ERROR_CODE ec, const std::string& msg);
    bool process_request(const std::string request_name, ResponseHandler handler, std::function<bool()> task_func);
    bool register_handler(const std::string& request_name, ResponseHandler handler);
    bool send_request(const std::string& request_name, const std::string& request, const std::string& uuid="");

    // 重构后的周期性任务
    PeriodicTask get_3dcamera_pointcloud_task_;
    PeriodicTask get_robot_state_task_;
    PeriodicTask get_model_polygon_task_;
    PeriodicTask get_obst_pcl_task_;
    PeriodicTask get_obst_polygon_task_;
    PeriodicTask get_scan2pointcloud_task_;
    PeriodicTask get_qr_camera_data_task_;
    PeriodicTask get_point_cloud_task_;
    PeriodicTask get_agv_position_task_;

    // 私有实现
    bool get_robot_state();
    bool get_model_polygon();
    bool get_obst_pcl();
    bool get_obst_polygon();
    bool get_scan2pointcloud();
    bool get_qr_camera_data();
    bool get_point_cloud();
    bool get_agv_position();

    // 辅助函数
    bool is_valid_port(const std::string& portStr);
    bool is_valid_ipv4(const std::string& ip);

public:
    // 操作模式常量
    static constexpr int automatic_mode     = 0;
    static constexpr int semiautomatic_mode = 1;
    static constexpr int manual_mode        = 2;
    static constexpr int service_mode       = 3;
    static constexpr int teaching_mode      = 4;
};
}   //namespace qclcpp
#endif // CLIENT_H