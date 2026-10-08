#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cstring>
#include <cstdint>
#include <thread>
#include <atomic>
#include <mutex>
#include <pthread.h>

#include <std_srvs/srv/trigger.hpp>
#include <array>
#include <condition_variable>

#include "soem/soem.h"

// ── EtherCAT slave indices ────────────────────────────────────────────────────
static constexpr int SLAVE_EK1100 = 1;
static constexpr int SLAVE_EL3102 = 2;
static constexpr int SLAVE_BOTA   = 3;

// ── IOmap size (bytes) ────────────────────────────────────────────────────────
static constexpr std::size_t IOMAP_SIZE = 4096;

// ── Bota SensONE PDO layout (reverse-engineered) ─────────────────────────────
#pragma pack(push, 1)
struct BotaPDO {
    uint8_t  status;
    uint32_t warnings;
    float    force_x;
    float    force_y;
    float    force_z;
    float    torque_x;
    float    torque_y;
    float    torque_z;
    uint16_t ft_saturated;
    float    accel_x;
    float    accel_y;
    float    accel_z;
    uint8_t  accel_saturated;
    float    gyro_x;
    float    gyro_y;
    float    gyro_z;
    uint8_t  gyro_saturated;
    float    temperature;
    float    orientation_x;
    float    orientation_y;
    float    orientation_z;
    float    orientation_w;
};
#pragma pack(pop)

// ─────────────────────────────────────────────────────────────────────────────

class EthercatPublisher : public rclcpp::Node
{
public:
    EthercatPublisher();
    ~EthercatPublisher();

private:
    // ── Initialisation ────────────────────────────────────────────────────────
    bool init_ethercat();

    // ── Cyclic RT loop ────────────────────────────────────────────────────────
    void cyclic_loop();

    // ── Per-frame processing ──────────────────────────────────────────────────
    void process_el3102(const rclcpp::Time & stamp);
    void process_bota  (const rclcpp::Time & stamp);

    // ── EtherCAT recovery helper ──────────────────────────────────────────────
    bool recover_slaves();
    void publish_cached_data();

    // ── Tare ─────────────────────────────────────────────────────────────────
    void on_tare(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_srv_;
    std::mutex              tare_mutex_;
    std::condition_variable tare_cv_;
    std::array<double, 6>   ft_offset_{};   // fx fy fz tx ty tz
    std::array<double, 6>   tare_sum_{};
    int                     tare_count_   = 0;
    int                     tare_samples_ = 200;
    bool                    tare_active_  = false;

    // ── Publishers ───────────────────────────────────────────────────────────
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr           pub_voltage_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_wrench_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr            pub_imu_;

    // ── SOEM context ─────────────────────────────────────────────────────────
    ecx_contextt ctx_;
    uint8_t      io_map_[IOMAP_SIZE];

    // ── RT thread ────────────────────────────────────────────────────────────
    std::thread       cyclic_thread_;
    std::atomic<bool> running_{false};

    // ── Parameters (set in constructor, read-only after that) ────────────────
    std::string nic_;            // network interface name
    std::string wrench_topic_;   // published wrench topic
    std::string imu_topic_;      // published imu topic
    std::string voltage_topic_;  // published voltage topic
    std::string wrench_frame_;   // frame_id for WrenchStamped
    std::string imu_frame_;      // frame_id for Imu
    
    std::mutex data_mutex_;

    geometry_msgs::msg::WrenchStamped latest_wrench_;
    sensor_msgs::msg::Imu latest_imu_;
    std_msgs::msg::Float64 latest_voltage_;

    rclcpp::TimerBase::SharedPtr pub_timer_;

    int expected_wkc_ = 0;

    bool ethercat_initialized_ = false;
    
    std::atomic<bool> data_ready_{false};
};
