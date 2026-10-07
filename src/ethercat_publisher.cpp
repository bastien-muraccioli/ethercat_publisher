#include "ethercat_publisher.h"

#include <sched.h>
#include <time.h>
#include <cmath>
#include <sys/mman.h>

// ── Constants ─────────────────────────────────────────────────────────────────
static constexpr long   CYCLE_NS        = 1'000'000L;   // 1 ms  → 1 kHz
static constexpr double ADC_SCALE       = 10.0 / 32767.0;
static constexpr double GRAVITY         = 9.81;
static constexpr int    SCHED_PRIORITY  = 80;
static constexpr int    MAX_LOST_FRAMES = 10;            // frames before recovery attempt

// ─────────────────────────────────────────────────────────────────────────────
EthercatPublisher::EthercatPublisher()
: Node("ethercat_publisher")
{
    // ── Declare + read ROS parameters ─────────────────────────────────────────
    declare_parameter("nic",           "enx00e04c68027b");
    declare_parameter("wrench_topic",  "/bus0/ft_sensor0/ft_sensor_readings/wrench");
    declare_parameter("imu_topic",     "/bus0/ft_sensor0/ft_sensor_readings/imu");
    declare_parameter("voltage_topic", "/collision/voltage");
    declare_parameter("wrench_frame",  "FT_sensor_wrench");
    declare_parameter("imu_frame",     "FT_sensor_imu");

    nic_           = get_parameter("nic").as_string();
    wrench_topic_  = get_parameter("wrench_topic").as_string();
    imu_topic_     = get_parameter("imu_topic").as_string();
    voltage_topic_ = get_parameter("voltage_topic").as_string();
    wrench_frame_  = get_parameter("wrench_frame").as_string();
    imu_frame_     = get_parameter("imu_frame").as_string();

    // ── Publishers ────────────────────────────────────────────────────────────
    auto qos = rclcpp::SensorDataQoS();   // best-effort, depth 10 — suits 1 kHz sensor data
    pub_voltage_ = create_publisher<std_msgs::msg::Float64>           (voltage_topic_, qos);
    pub_wrench_  = create_publisher<geometry_msgs::msg::WrenchStamped>(wrench_topic_,  qos);
    pub_imu_     = create_publisher<sensor_msgs::msg::Imu>            (imu_topic_,     qos);

    // ── EtherCAT init ─────────────────────────────────────────────────────────
    RCLCPP_INFO(get_logger(), "Initializing EtherCAT on %s ...", nic_.c_str());
    if (!init_ethercat()) {
        RCLCPP_FATAL(get_logger(), "EtherCAT initialisation failed — node will not run");
        return;
    }
    RCLCPP_INFO(get_logger(), "EtherCAT ready, starting 1 kHz cyclic loop");

    // ── Start RT thread ───────────────────────────────────────────────────────
    pub_timer_ = create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&EthercatPublisher::publish_cached_data, this));
        
    running_ = true;
    cyclic_thread_ = std::thread(&EthercatPublisher::cyclic_loop, this);
}

// ─────────────────────────────────────────────────────────────────────────────
EthercatPublisher::~EthercatPublisher()
{
    running_ = false;
    if (cyclic_thread_.joinable())
        cyclic_thread_.join();

    if (ethercat_initialized_) {
        ctx_.slavelist[0].state = EC_STATE_INIT;
        ecx_writestate(&ctx_, 0);
        ecx_close(&ctx_);
        RCLCPP_INFO(get_logger(), "EtherCAT closed cleanly");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
bool EthercatPublisher::init_ethercat()
{
    memset(&ctx_,    0, sizeof(ctx_));
    memset(io_map_,  0, sizeof(io_map_));

    if (!ecx_init(&ctx_, nic_.c_str())) {
        RCLCPP_ERROR(get_logger(), "ecx_init failed on interface '%s'", nic_.c_str());
        return false;
    }

    int n = ecx_config_init(&ctx_);
    if (n <= 0) {
        RCLCPP_ERROR(get_logger(), "No EtherCAT slaves found");
        return false;
    }
    RCLCPP_INFO(get_logger(), "%d slave(s) found", n);

    // Verify expected topology
    if (n < 3) {
        RCLCPP_ERROR(get_logger(),
            "Expected 3 slaves (EK1100, EL3102, Bota), found %d", n);
        return false;
    }

    ecx_config_map_group(&ctx_, io_map_, 0);
    ecx_configdc(&ctx_);
    expected_wkc_ =
      (ctx_.grouplist[0].outputsWKC * 2) +
       ctx_.grouplist[0].inputsWKC;

    // Request SAFE-OP
    ecx_statecheck(&ctx_, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    // One warm-up exchange before going OP
    ecx_send_processdata(&ctx_);
    ecx_receive_processdata(&ctx_, EC_TIMEOUTRET);

    // Request OP
    ctx_.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ctx_, 0);

    int state = ecx_statecheck(&ctx_, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    if (state != EC_STATE_OPERATIONAL) {
        RCLCPP_ERROR(get_logger(),
            "Slaves did not reach OP state (state=0x%02X)", state);
        return false;
    }

    ethercat_initialized_ = true;

    RCLCPP_INFO(get_logger(), "All slaves in OPERATIONAL state");
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
void EthercatPublisher::cyclic_loop()
{
    mlockall(MCL_CURRENT | MCL_FUTURE);
    // ── Elevate thread to SCHED_FIFO ─────────────────────────────────────────
    sched_param sp{};
    sp.sched_priority = SCHED_PRIORITY;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
        RCLCPP_WARN(get_logger(),
            "Could not set SCHED_FIFO (run with sudo or set cap_sys_nice). "
            "Timing may be less precise.");
    }

    // ── Absolute-time loop anchor ─────────────────────────────────────────────
    struct timespec next{};
    clock_gettime(CLOCK_MONOTONIC, &next);

    int lost_frames = 0;

    while (running_) {
        // ── Advance deadline by one cycle ─────────────────────────────────────
        next.tv_nsec += CYCLE_NS;
        if (next.tv_nsec >= 1'000'000'000L) {
            next.tv_nsec -= 1'000'000'000L;
            next.tv_sec  += 1;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);

        // ── EtherCAT frame exchange ───────────────────────────────────────────
        ecx_send_processdata(&ctx_);
        int wkc = ecx_receive_processdata(&ctx_, EC_TIMEOUTRET);

        // ── Working counter check — detect lost frames ────────────────────────
        if (wkc < expected_wkc_) {
            if (++lost_frames >= MAX_LOST_FRAMES) {
                RCLCPP_WARN(get_logger(),
                    "Lost %d consecutive frames (wkc=%d expected=%d) — attempting recovery",
                    lost_frames, wkc, (ctx_.grouplist[0].outputsWKC + ctx_.grouplist[0].inputsWKC));
                if (recover_slaves())
                    lost_frames = 0;
            }
            continue;   // skip publishing on bad frame
        }
        lost_frames = 0;

        // ── Timestamp ─────────────────────────────────────────────────────────
        auto stamp = now();

        // ── Cache latest sensor data ──────────────────────────────────────────
        process_el3102(stamp);
        process_bota(stamp);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void EthercatPublisher::process_el3102(const rclcpp::Time & stamp)
{
    (void)stamp;

    uint8_t *data =
        reinterpret_cast<uint8_t*>(
            ctx_.slavelist[SLAVE_EL3102].inputs);

    // EL3102 actual layout:
    // byte1 = low byte
    // byte2 = high byte
    int16_t raw =
        static_cast<int16_t>(
            (data[2] << 8) | data[1]);

    double voltage =
        (static_cast<double>(raw) / 32767.0) * 10.0;

    std_msgs::msg::Float64 msg;
    msg.data = voltage;

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_voltage_ = msg;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void EthercatPublisher::process_bota(const rclcpp::Time & stamp)
{
    const auto * pdo = reinterpret_cast<const BotaPDO *>(
        ctx_.slavelist[SLAVE_BOTA].inputs);

    // ── WrenchStamped ─────────────────────────────────────────────────────────
    geometry_msgs::msg::WrenchStamped wrench;
    wrench.header.stamp    = stamp;
    wrench.header.frame_id = wrench_frame_;
    wrench.wrench.force.x  = pdo->force_x;
    wrench.wrench.force.y  = pdo->force_y;
    wrench.wrench.force.z  = pdo->force_z;
    wrench.wrench.torque.x = pdo->torque_x;
    wrench.wrench.torque.y = pdo->torque_y;
    wrench.wrench.torque.z = pdo->torque_z;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_wrench_ = wrench;  
    }

    // ── Imu ───────────────────────────────────────────────────────────────────
    sensor_msgs::msg::Imu imu;
    imu.header.stamp    = stamp;
    imu.header.frame_id = imu_frame_;

    // Linear acceleration: sensor reports in g → convert to m/s²
    imu.linear_acceleration.x = pdo->accel_x * GRAVITY;
    imu.linear_acceleration.y = pdo->accel_y * GRAVITY;
    imu.linear_acceleration.z = pdo->accel_z * GRAVITY;

    // Angular velocity (rad/s — already SI from Bota)
    imu.angular_velocity.x = pdo->gyro_x;
    imu.angular_velocity.y = pdo->gyro_y;
    imu.angular_velocity.z = pdo->gyro_z;

    // Orientation quaternion
    imu.orientation.x = pdo->orientation_x;
    imu.orientation.y = pdo->orientation_y;
    imu.orientation.z = pdo->orientation_z;
    imu.orientation.w = pdo->orientation_w;

    // Covariance: -1 in [0] signals "not provided" per REP-145
    imu.orientation_covariance[0]         = -1.0;
    imu.angular_velocity_covariance[0]    = -1.0;
    imu.linear_acceleration_covariance[0] = -1.0;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_imu_ = imu;  
    }
    
    data_ready_ = true;
}

void EthercatPublisher::publish_cached_data()
{
    if (!data_ready_)
        return;

    std::lock_guard<std::mutex> lock(data_mutex_);

    pub_voltage_->publish(latest_voltage_);
    pub_wrench_->publish(latest_wrench_);
    pub_imu_->publish(latest_imu_);
}

// ─────────────────────────────────────────────────────────────────────────────
bool EthercatPublisher::recover_slaves()
{
    // Re-request OPERATIONAL for any slave that dropped out
    for (int i = 1; i <= ctx_.slavecount; ++i) {
        if (ctx_.slavelist[i].state != EC_STATE_OPERATIONAL) {

            RCLCPP_WARN(get_logger(),
                "Slave %d not in OP (state=0x%02X) — recovering",
                i,
                ctx_.slavelist[i].state);

            if (ctx_.slavelist[i].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {

                ctx_.slavelist[i].state = EC_STATE_SAFE_OP + EC_STATE_ACK;
                ecx_writestate(&ctx_, i);
            }
            else if (ctx_.slavelist[i].state == EC_STATE_SAFE_OP) {

                ctx_.slavelist[i].state = EC_STATE_OPERATIONAL;
                ecx_writestate(&ctx_, i);
            }
            else if (ctx_.slavelist[i].state == 0) {

                ecx_recover_slave(&ctx_, i, EC_TIMEOUTRET);
            }
        }
    }
    int state = ecx_statecheck(&ctx_, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    if (state == EC_STATE_OPERATIONAL) {
        RCLCPP_INFO(get_logger(), "All slaves recovered to OPERATIONAL");
        return true;
    }
    RCLCPP_ERROR(get_logger(), "Recovery failed — slaves still not in OP (state=0x%02X)", state);
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EthercatPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
