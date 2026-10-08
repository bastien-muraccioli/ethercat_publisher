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
    declare_parameter("nic",           "enp45s0");
    declare_parameter("wrench_topic",  "/bus0/ft_sensor0/ft_sensor_readings/wrench");
    declare_parameter("imu_topic",     "/bus0/ft_sensor0/ft_sensor_readings/imu");
    declare_parameter("voltage_topic", "/collision/voltage");
    declare_parameter("wrench_frame",  "FT_sensor_wrench");
    declare_parameter("imu_frame",     "FT_sensor_imu");
    declare_parameter("tare_service", "/bota_ft_sensor/tare");
    declare_parameter("tare_samples", 200);   // 200 samples = 0.2 s at 1 kHz

    // Bota filter settings written at startup (-1 = keep what is stored on the sensor)
    declare_parameter("bota_sinc_length", 51);   // rate ≈ 51200/sinc: 51 = 1 kHz, 64 = 800 Hz, 128 = 400 Hz. -1 = keep sensor value
    declare_parameter("bota_fir_disable", -1);   // 1 = FIR off, 0 = FIR on
    declare_parameter("bota_fast_enable", -1);   // 1 = FAST (spike) filter on
    declare_parameter("bota_chop_enable", -1);   // 1 = CHOP on
    declare_parameter("report_update_rate", true);
    report_update_rate_ = get_parameter("report_update_rate").as_bool();
    tare_samples_ = get_parameter("tare_samples").as_int();

    tare_srv_ = create_service<std_srvs::srv::Trigger>(
        get_parameter("tare_service").as_string(),
        std::bind(&EthercatPublisher::on_tare, this,
                  std::placeholders::_1, std::placeholders::_2));

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

    // Slaves are in PRE-OP here: mailbox (SDO) access works, PDOs not mapped yet.
    // ecx_config_map_group() below requests SAFE-OP, so configure before it.
    configure_bota();

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
    auto rate_t0 = std::chrono::steady_clock::now();

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

        // ── Report real sensor update rate every 5 s ──────────────────────────
        const auto t  = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(t - rate_t0).count();
        if (dt >= 5.0) {
            if (report_update_rate_) {
                RCLCPP_INFO(get_logger(),
                    "EtherCAT loop %.0f Hz | new Bota F/T samples %.0f Hz",
                    ft_frames_ / dt, ft_changes_ / dt);
            }
            ft_frames_ = ft_changes_ = 0;
            rate_t0 = t;
        }
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

    const std::array<double, 6> raw = {
        pdo->force_x,  pdo->force_y,  pdo->force_z,
        pdo->torque_x, pdo->torque_y, pdo->torque_z };

    // A changed value means the sensor produced a new sample since the last frame
    ++ft_frames_;
    if (raw != last_ft_raw_) {
        ++ft_changes_;
        last_ft_raw_ = raw;
    }

    std::array<double, 6> off;
    {
        std::lock_guard<std::mutex> lk(tare_mutex_);
        if (tare_active_) {
            for (size_t i = 0; i < 6; ++i) tare_sum_[i] += raw[i];
            if (++tare_count_ >= tare_samples_) {
                for (size_t i = 0; i < 6; ++i) ft_offset_[i] = tare_sum_[i] / tare_count_;
                tare_active_ = false;
                tare_cv_.notify_all();
            }
        }
        off = ft_offset_;
    }

    wrench.wrench.force.x  = raw[0] - off[0];
    wrench.wrench.force.y  = raw[1] - off[1];
    wrench.wrench.force.z  = raw[2] - off[2];
    wrench.wrench.torque.x = raw[3] - off[3];
    wrench.wrench.torque.y = raw[4] - off[4];
    wrench.wrench.torque.z = raw[5] - off[5];
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
// ── Bota SDO configuration ───────────────────────────────────────────────────
// Object dictionary (Bota F/T sensor user manual, EtherCAT SDO table):
//   0x8006:01 Sinc length   0x8006:02 FIR disable   0x8006:03 FAST enable   0x8006:04 CHOP enable
//   0x8010:01 Calibration matrix active   0x8010:02 Temperature compensation   0x8010:03 IMU active
//   0x8011:00 Sampling rate [Hz] (read-only, reflects the filter settings)
//   0x8030:01 Control command (0x01 = save parameters to flash) — NOT used here
static constexpr uint16_t OD_FT_FILTER     = 0x8006;
static constexpr uint16_t OD_DEVICE_CONFIG = 0x8010;
static constexpr uint16_t OD_SAMPLING_RATE = 0x8011;

bool EthercatPublisher::sdo_read(uint16_t index, uint8_t sub, uint32_t & value)
{
    uint8_t buf[4] = {0, 0, 0, 0};
    int size = sizeof(buf);
    const int wkc = ecx_SDOread(&ctx_, SLAVE_BOTA, index, sub, false,
                                &size, buf, EC_TIMEOUTRXM);
    if (wkc <= 0)
        return false;
    value = 0;
    for (int i = 0; i < size && i < 4; ++i)       // little-endian, any width ≤ 4
        value |= static_cast<uint32_t>(buf[i]) << (8 * i);
    return true;
}

bool EthercatPublisher::sdo_write(uint16_t index, uint8_t sub, uint32_t value, int size)
{
    uint8_t buf[4];
    for (int i = 0; i < 4; ++i)
        buf[i] = static_cast<uint8_t>(value >> (8 * i));
    const int wkc = ecx_SDOwrite(&ctx_, SLAVE_BOTA, index, sub, false,
                                 size, buf, EC_TIMEOUTRXM);
    if (wkc <= 0) {
        while (ecx_iserror(&ctx_))
            RCLCPP_ERROR(get_logger(), "SDO error: %s", ecx_elist2string(&ctx_));
        return false;
    }
    return true;
}

void EthercatPublisher::configure_bota()
{
    const auto & s = ctx_.slavelist[SLAVE_BOTA];

    // ecx_config_init() only *requests* PRE-OP. SOEM 2.x refuses mailbox (SDO)
    // traffic until the slave's cached state is >= PRE-OP, so wait for it here.
    const uint16_t st = ecx_statecheck(&ctx_, SLAVE_BOTA, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    if ((st & 0x0F) < EC_STATE_PRE_OP) {
        RCLCPP_ERROR(get_logger(),
            "Bota did not reach PRE-OP (state=0x%02X): cannot read/write its configuration", st);
        return;
    }

    RCLCPP_INFO(get_logger(), "Slave %d: '%s' (vendor 0x%08X, product 0x%08X)",
        SLAVE_BOTA, s.name, static_cast<unsigned>(s.eep_man), static_cast<unsigned>(s.eep_id));

    struct Setting { const char * param; uint8_t sub; int size; };
    const Setting settings[] = {
        {"bota_sinc_length", 0x01, 2},
        {"bota_fir_disable", 0x02, 1},
        {"bota_fast_enable", 0x03, 1},
        {"bota_chop_enable", 0x04, 1},
    };

    bool wrote = false;
    for (const auto & st : settings) {
        const int64_t v = get_parameter(st.param).as_int();
        if (v < 0)
            continue;   // keep the value stored on the sensor
        if (st.sub == 0x01 && v != 51 && v != 64 && v != 128 &&
            v != 205 && v != 256 && v != 512) {
            RCLCPP_WARN(get_logger(), "%s=%ld is not a documented value "
                "(51, 64, 128, 205, 256, 512), skipping", st.param, static_cast<long>(v));
            continue;
        }
        if (sdo_write(OD_FT_FILTER, st.sub, static_cast<uint32_t>(v), st.size)) {
            RCLCPP_INFO(get_logger(), "Set %s = %ld", st.param, static_cast<long>(v));
            wrote = true;
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to set %s", st.param);
        }
    }
    if (wrote)   // give the sensor time to apply the new filter before reading back
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

    log_bota_config();

    // The cyclic loop runs at 1 kHz: warn if the sensor produces fewer samples
    uint32_t rate = 0;
    if (sdo_read(0x8011, 0x00, rate) && rate < 1000) {
        RCLCPP_WARN(get_logger(),
            "Bota sampling rate is %u Hz (< 1 kHz): consecutive messages will repeat "
            "the same sample. Use bota_sinc_length:=51 for 1 kHz (rate ~ 51200 / sinc).", rate);
    }
}

void EthercatPublisher::log_bota_config()
{
    auto rd = [this](uint16_t idx, uint8_t sub) -> std::string {
        uint32_t v = 0;
        return sdo_read(idx, sub, v) ? std::to_string(v) : std::string("?");
    };

    RCLCPP_INFO(get_logger(),
        "Bota config: sinc length=%s, FIR disable=%s, FAST=%s, CHOP=%s | "
        "calibration=%s, temp. comp.=%s, IMU=%s | sampling rate=%s Hz",
        rd(OD_FT_FILTER, 0x01).c_str(), rd(OD_FT_FILTER, 0x02).c_str(),
        rd(OD_FT_FILTER, 0x03).c_str(), rd(OD_FT_FILTER, 0x04).c_str(),
        rd(OD_DEVICE_CONFIG, 0x01).c_str(), rd(OD_DEVICE_CONFIG, 0x02).c_str(),
        rd(OD_DEVICE_CONFIG, 0x03).c_str(),
        rd(OD_SAMPLING_RATE, 0x00).c_str());
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

void EthercatPublisher::on_tare(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (!ethercat_initialized_) {
        res->success = false;
        res->message = "EtherCAT not running";
        return;
    }

    std::unique_lock<std::mutex> lk(tare_mutex_);
    tare_sum_.fill(0.0);
    tare_count_  = 0;
    tare_active_ = true;

    // The cyclic thread fills the samples; wait for it (with a timeout)
    const bool done = tare_cv_.wait_for(lk, std::chrono::seconds(2),
                                        [this] { return !tare_active_; });
    if (!done) {
        tare_active_ = false;
        res->success = false;
        res->message = "Tare timed out: no valid sensor frames";
        return;
    }

    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "Tared over %d samples. Offset F=[%.3f %.3f %.3f] T=[%.4f %.4f %.4f]",
        tare_count_, ft_offset_[0], ft_offset_[1], ft_offset_[2],
        ft_offset_[3], ft_offset_[4], ft_offset_[5]);
    res->success = true;
    res->message = buf;
    RCLCPP_INFO(get_logger(), "%s", buf);
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
