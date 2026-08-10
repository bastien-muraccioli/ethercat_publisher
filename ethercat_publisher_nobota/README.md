# ethercat_publisher

ROS 2 EtherCAT driver for a Kinova arm setup with:
- **EK1100** — EtherCAT coupler (slave 1)
- **EL3102** — 2-channel analog input terminal / ADC (slave 2)
- **Bota SensONE** — force-torque + IMU sensor (slave 3)

This node is the sole EtherCAT master on the bus. It reads both the EL3102
and Bota PDOs in a single 1 kHz cyclic frame and publishes the data to ROS 2
topics consumed by the `RosForceSensor` and `RosImuSensor` mc_rtc plugins.

---

## Data flow

```
ethercat_publisher_node
    │
    ├── /bus0/ft_sensor0/ft_sensor_readings/wrench  (geometry_msgs/WrenchStamped)
    │         └── mc_rtc RosForceSensor plugin → "EEForceSensor"
    │
    ├── /bus0/ft_sensor0/ft_sensor_readings/imu     (sensor_msgs/Imu)
    │         └── mc_rtc RosImuSensor plugin  → "Accelerometer"
    │
    └── /collision/voltage                           (std_msgs/Float64)
              └── mc_rtc ADCCollisionSensor plugin  → LpfThreshold filter
```

---

## Dependencies

### ROS 2 packages
| Package | Notes |
|---|---|
| `rclcpp` | ROS 2 C++ client library |
| `std_msgs` | Float64 message |
| `sensor_msgs` | Imu message |
| `geometry_msgs` | WrenchStamped message |

### SOEM (Simple Open EtherCAT Master)
SOEM must be built from source. It is **not** available as a Jazzy apt package.

```bash
git clone https://github.com/OpenEtherCATsociety/SOEM.git ~/SOEM
cd ~/SOEM && mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

By default CMake looks for SOEM at `~/SOEM`. To use a different path:
```bash
colcon build --cmake-args -DSOEM_DIR=/path/to/your/SOEM
```

---

## Building

```bash
cd ~/bota_ws
colcon build --packages-select ethercat_publisher \ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```
Important
- Build without `--symlink-install`.
- Re-apply capabilities after every rebuild.


---

## Capabilities (required once after every rebuild)

The node requires:
- raw socket access for SOEM,
- network administration capability for EtherCAT NIC access,
- real-time scheduling for the 1 kHz cyclic loop.

Grant capabilities to the executable after every rebuild:

```bash
sudo setcap cap_net_raw,cap_net_admin,cap_sys_nice=+ep \
  ~/bota_ws/install/ethercat_publisher/lib/ethercat_publisher/ethercat_publisher_node
```

Verify:

```bash
getcap \
  ~/bota_ws/install/ethercat_publisher/lib/ethercat_publisher/ethercat_publisher_node
```

Expected output:
```bash
cap_net_admin,cap_net_raw,cap_sys_nice=ep
```

Notes:
- Rebuilding the package removes Linux capabilities because the executable is recreated.
- Do NOT use `colcon build --symlink-install` for this package, as Linux capabilities may not propagate correctly through symlinks.

---

## Running

```bash
ros2 launch ethercat_publisher ethercat_publisher.launch.py
```

### Verifying output
```bash
ros2 topic hz /bus0/ft_sensor0/ft_sensor_readings/wrench   # expect ~1000 Hz
ros2 topic hz /collision/voltage                            # expect ~1000 Hz
ros2 topic echo /collision/voltage --once
```

---

## Parameters

All parameters can be overridden at launch time:

| Parameter | Default | Description |
|---|---|---|
| `nic` | `enx00e04c68027b` | Network interface for EtherCAT |
| `wrench_topic` | `/bus0/ft_sensor0/ft_sensor_readings/wrench` | WrenchStamped topic |
| `imu_topic` | `/bus0/ft_sensor0/ft_sensor_readings/imu` | Imu topic |
| `voltage_topic` | `/collision/voltage` | ADC voltage topic |
| `wrench_frame` | `FT_sensor_wrench` | frame_id for WrenchStamped |
| `imu_frame` | `FT_sensor_imu` | frame_id for Imu |

Example override:
```bash
ros2 launch ethercat_publisher ethercat_publisher.launch.py \
  nic:=eth0
```

---

## EtherCAT topology

```
PC (enx00e04c68027b)
  └── EK1100  [slave 1] — coupler
        ├── EL3102  [slave 2] — 2ch ADC, ±10V, channel 1 used
        └── Bota SensONE  [slave 3] — FT + IMU
```

## Notes

- The node operates as the sole EtherCAT master on the bus.
- `libBotaDriverExposed.so` from `bota_driver_ros2` is installed on the system but never launched.
- Do not run `bota_driver_node` or any second EtherCAT master simultaneously.
- The cyclic EtherCAT loop runs at 1 kHz using `clock_nanosleep` with `SCHED_FIFO`.
- ROS 2 publishing is decoupled from the EtherCAT real-time thread through cached messages and a ROS timer publisher.
- Slave recovery is automatically attempted if working counter mismatches persist.
- The EL3102 PDO layout is decoded manually from raw EtherCAT bytes because Beckhoff compact PDO mapping does not match the default packed struct layout.
- EL3102 voltage conversion:
`voltage = (raw_int16 / 32767.0) * 10.0`
- The Bota SensONE PDO layout was reverse-engineered from raw EtherCAT traffic.
