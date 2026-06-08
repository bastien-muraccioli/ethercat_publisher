#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cstring>
#include <cstdint>
#include <thread>
#include <atomic>
#include <cmath>

#include "soem/soem.h"

#define EK1100_SLAVE_IDX 1
#define EL3102_SLAVE_IDX 2
#define BOTA_SLAVE_IDX   3
#define NETWORK_INTERFACE "enx00e04c68027b"
#define COLLISION_THRESHOLD 5.0

#pragma pack(push, 1)
struct EL3102PDO {
    uint8_t  status_ch1;
    int16_t  value_ch1;
    uint8_t  status_ch2;
    int16_t  value_ch2;
};
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

class EthercatPublisher : public rclcpp::Node {
public:
    EthercatPublisher() : Node("ethercat_publisher"), running_(false) {
        auto qos = rclcpp::QoS(10).reliable();
        pub_collision_ = create_publisher<std_msgs::msg::Bool>("/collision", qos);
        pub_voltage_   = create_publisher<std_msgs::msg::Float64>("/collision/voltage", qos);
        pub_imu_       = create_publisher<sensor_msgs::msg::Imu>("/bota/imu", qos);
        pub_wrench_    = create_publisher<geometry_msgs::msg::WrenchStamped>("/bota/wrench", qos);

        RCLCPP_INFO(get_logger(), "Initializing EtherCAT...");
        if(!init_ethercat()) {
            RCLCPP_FATAL(get_logger(), "EtherCAT initialization failed");
            return;
        }
        RCLCPP_INFO(get_logger(), "EtherCAT initialized, starting read loop");
        running_ = true;
        ethercat_thread_ = std::thread(&EthercatPublisher::read_loop, this);
    }

    ~EthercatPublisher() {
        running_ = false;
        if(ethercat_thread_.joinable())
            ethercat_thread_.join();
        ecx_close(&ctx_);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_collision_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_voltage_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_wrench_;

    ecx_contextt ctx_;
    uint8_t io_map_[4096];
    std::thread ethercat_thread_;
    std::atomic<bool> running_;
    int contact_prev_ = 0;

    bool init_ethercat() {
        memset(&ctx_, 0, sizeof(ctx_));
        if(!ecx_init(&ctx_, NETWORK_INTERFACE)) {
            RCLCPP_ERROR(get_logger(), "Failed to init on %s", NETWORK_INTERFACE);
            return false;
        }
        int slave_count = ecx_config_init(&ctx_);
        if(slave_count <= 0) {
            RCLCPP_ERROR(get_logger(), "No slaves found");
            return false;
        }
        RCLCPP_INFO(get_logger(), "%d slaves found", slave_count);
        ecx_config_map_group(&ctx_, io_map_, 0);
        ecx_configdc(&ctx_);
        ecx_statecheck(&ctx_, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
        ecx_send_processdata(&ctx_);
        ecx_receive_processdata(&ctx_, EC_TIMEOUTRET);
        ctx_.slavelist[0].state = EC_STATE_OPERATIONAL;
        ecx_writestate(&ctx_, 0);
        ecx_statecheck(&ctx_, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
        RCLCPP_INFO(get_logger(), "All slaves operational");
        return true;
    }

    void read_loop() {
        while(running_) {
            ecx_send_processdata(&ctx_);
            ecx_receive_processdata(&ctx_, EC_TIMEOUTRET);

            auto stamp = now();

            // ── EL3102 ───────────────────────────────────────
            uint8_t *el_data = ctx_.slavelist[EL3102_SLAVE_IDX].inputs;
            EL3102PDO *el = reinterpret_cast<EL3102PDO*>(el_data);
            double voltage = (el->value_ch1 / 32767.0) * 10.0;
            int contact_now = (voltage > COLLISION_THRESHOLD) ? 1 : 0;

            auto volt_msg = std_msgs::msg::Float64();
            volt_msg.data = voltage;
            pub_voltage_->publish(volt_msg);

            auto bool_msg = std_msgs::msg::Bool();
            bool_msg.data = contact_now;
            pub_collision_->publish(bool_msg);

            if(contact_now == 1 && contact_prev_ == 0)
                RCLCPP_INFO(get_logger(), "*** COLLISION DETECTED *** %.3fV", voltage);
            contact_prev_ = contact_now;

            // ── Bota SensONE ─────────────────────────────────
            uint8_t *bota_data = ctx_.slavelist[BOTA_SLAVE_IDX].inputs;
            BotaPDO *bota = reinterpret_cast<BotaPDO*>(bota_data);

            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.header.stamp = stamp;
            imu_msg.header.frame_id = "bota_ft_sensor";
            imu_msg.linear_acceleration.x = bota->accel_x * 9.81;
            imu_msg.linear_acceleration.y = bota->accel_y * 9.81;
            imu_msg.linear_acceleration.z = bota->accel_z * 9.81;
            imu_msg.angular_velocity.x = bota->gyro_x;
            imu_msg.angular_velocity.y = bota->gyro_y;
            imu_msg.angular_velocity.z = bota->gyro_z;
            imu_msg.orientation.x = bota->orientation_x;
            imu_msg.orientation.y = bota->orientation_y;
            imu_msg.orientation.z = bota->orientation_z;
            imu_msg.orientation.w = bota->orientation_w;
            imu_msg.orientation_covariance[0] = -1;
            imu_msg.angular_velocity_covariance[0] = -1;
            imu_msg.linear_acceleration_covariance[0] = -1;
            pub_imu_->publish(imu_msg);

            auto wrench_msg = geometry_msgs::msg::WrenchStamped();
            wrench_msg.header.stamp = stamp;
            wrench_msg.header.frame_id = "bota_ft_sensor";
            wrench_msg.wrench.force.x = bota->force_x;
            wrench_msg.wrench.force.y = bota->force_y;
            wrench_msg.wrench.force.z = bota->force_z;
            wrench_msg.wrench.torque.x = bota->torque_x;
            wrench_msg.wrench.torque.y = bota->torque_y;
            wrench_msg.wrench.torque.z = bota->torque_z;
            pub_wrench_->publish(wrench_msg);

            osal_usleep(1000);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EthercatPublisher>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
