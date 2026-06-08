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
colcon build --packages-select ethercat_publisher \
             --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

---

## Capabilities (required once after every rebuild)

The node needs raw socket access and real-time scheduling. Grant both without
running as root:

```bash
sudo setcap cap_net_raw,cap_sys_nice=+ep \
  ~/bota_ws/install/ethercat_publisher/lib/ethercat_publisher/ethercat_publisher_node
```

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

- The Bota PDO layout was reverse-engineered from the raw EtherCAT frames.
  `libBotaDriverExposed.so` (from `bota_driver_ros2`) is **not** used — it is
  installed on the system but never launched.
- Only one process may call `ecx_init` on the NIC at a time. Do not run
  `bota_driver_node` alongside this node.
- EL3102 voltage conversion: `voltage = (raw_int16 / 32767.0) * 10.0`
