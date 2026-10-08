# ethercat_publisher

ROS 2 EtherCAT master node for a Kinova arm setup with:

| Slave | Device | Role |
|---|---|---|
| 1 | **Beckhoff EK1100** | EtherCAT coupler |
| 2 | **Beckhoff EL3102** | 2-channel ±10 V analog input (ADC), channel 1 used |
| 3 | **Bota SensONE** | 6-axis force/torque sensor + IMU |

The node is the **only** EtherCAT master on the bus. It exchanges one cyclic
frame at **1 kHz**, decodes the EL3102 and Bota PDOs, and publishes them to
ROS 2 topics consumed by the `RosForceSensor`, `RosImuSensor` and
`ADCCollisionSensor` mc_rtc plugins. It also provides a software **tare**
service for the force/torque sensor.

---

## Contents

- [Data flow](#data-flow)
- [Installation](#installation)
- [Network setup](#network-setup)
- [Capabilities](#capabilities-after-every-build)
- [Running](#running)
- [Tare / calibration](#tare--calibration)
- [Shell aliases](#shell-aliases)
- [Parameters](#parameters)
- [Bota sensor configuration](#bota-sensor-configuration)
- [Troubleshooting](#troubleshooting)
- [Implementation notes](#implementation-notes)

---

## Data flow

```
ethercat_publisher_node
    │
    ├── /bus0/ft_sensor0/ft_sensor_readings/wrench  (geometry_msgs/WrenchStamped, tared)
    │         └── mc_rtc RosForceSensor plugin → "EEForceSensor"
    │
    ├── /bus0/ft_sensor0/ft_sensor_readings/imu     (sensor_msgs/Imu)
    │         └── mc_rtc RosImuSensor plugin  → "Accelerometer"
    │
    ├── /collision/voltage                           (std_msgs/Float64)
    │         └── mc_rtc ADCCollisionSensor plugin → LpfThreshold filter
    │
    └── /bota_ft_sensor/tare                         (std_srvs/Trigger service)
```

EtherCAT topology:

```
PC (dedicated NIC, e.g. enp45s0)
  └── EK1100  [slave 1]  ← cable into X1 (IN)
        ├── EL3102        [slave 2]
        └── Bota SensONE  [slave 3]
```

---

## Installation

### Dependencies

| Dependency | Notes |
|---|---|
| ROS 2 (Jazzy) | `rclcpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `std_srvs` |
| CMake ≥ 3.28 | Required by SOEM 2.x (Ubuntu 24.04 ships it) |
| [SOEM](https://github.com/OpenEtherCATsociety/SOEM) ≥ 2.0 | **Handled automatically**, see below |

**SOEM does not need to be installed by hand.** At configure time CMake:

1. uses an installed SOEM ≥ 2.0 if it finds one (via `CMAKE_PREFIX_PATH` or `-Dsoem_DIR=<prefix>/cmake`);
2. otherwise downloads SOEM `v2.0.0` and builds it statically into the node.

> Do not use the `ros-<distro>-soem` apt package: it is the old 1.x API and will not compile with this code.

| CMake option | Default | Description |
|---|---|---|
| `ETHERCAT_PUBLISHER_FETCH_SOEM` | `ON` | Download SOEM if no suitable install is found |
| `SOEM_GIT_TAG` | `v2.0.0` | SOEM version to download |

### Option A: mc-rtc-superbuild

Add the project to your superbuild extension:

```cmake
AddCatkinProject(
  ethercat_publisher
  GITHUB_PRIVATE bastien-muraccioli/ethercat_publisher
  GIT_TAG origin/main
  WORKSPACE data_ws
)
```

The node is then installed in
`~/workspace/src/catkin_data_ws/install/lib/ethercat_publisher/ethercat_publisher_node`.

### Option B: standalone colcon workspace

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/bastien-muraccioli/ethercat_publisher.git
cd ~/ros2_ws
colcon build --packages-select ethercat_publisher --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

The node is then installed in
`~/ros2_ws/install/ethercat_publisher/lib/ethercat_publisher/ethercat_publisher_node`.

> Build **without** `--symlink-install`: capabilities are set on the real
> executable and are not reliable through symlinks.

---

## Network setup

The EtherCAT bus needs a **dedicated** Ethernet port. Do not use the port that
connects to the robot or your LAN. A port that has an IP address like
`192.168.1.x` is a normal IP network, not the EtherCAT chain.

```bash
ip -br link                                  # find the port cabled to the EK1100
sudo ip link set <iface> up
nmcli device set <iface> managed no          # keep NetworkManager (DHCP) off it
```

Then set `nic` to that interface (see [Parameters](#parameters)).

---

## Capabilities (after every build)

SOEM needs raw socket access, and the cyclic loop uses real-time scheduling.
Instead of running as root, grant the executable these capabilities:

| Capability | Why |
|---|---|
| `cap_net_raw` | Raw Ethernet socket for EtherCAT frames |
| `cap_net_admin` | Configure the NIC (promiscuous mode) |
| `cap_sys_nice` | `SCHED_FIFO` priority for the 1 kHz loop |

```bash
NODE=~/workspace/src/catkin_data_ws/install/lib/ethercat_publisher/ethercat_publisher_node   # superbuild
# NODE=~/ros2_ws/install/ethercat_publisher/lib/ethercat_publisher/ethercat_publisher_node   # colcon

sudo setcap cap_net_raw,cap_net_admin,cap_sys_nice=+ep $NODE
getcap $NODE
# expected: ... cap_net_admin,cap_net_raw,cap_sys_nice=ep
```

> ⚠️ **Every rebuild or reinstall erases these capabilities.** If the node
> prints `ecx_init failed on interface ...` right after a build, this is
> almost always the cause. Run `init_adc_bota` (see [aliases](#shell-aliases)).

---

## Running

```bash
ros2 launch ethercat_publisher ethercat_publisher.launch.py
```

A successful start looks like:

```
Initializing EtherCAT on enp45s0 ...
3 slave(s) found
All slaves in OPERATIONAL state
EtherCAT ready, starting 1 kHz cyclic loop
```

Check the output:

```bash
ros2 topic hz /bus0/ft_sensor0/ft_sensor_readings/wrench   # ~1000 Hz
ros2 topic hz /collision/voltage                            # ~1000 Hz
ros2 topic echo /collision/voltage --once
```

---

## Tare / calibration

The node provides a software tare. On request, it averages `tare_samples`
force/torque readings (default 200 samples, i.e. 0.2 s) and subtracts that
offset from every published wrench.

```bash
ros2 service call /bota_ft_sensor/tare std_srvs/srv/Trigger {}
```

```
response:
  success: true
  message: 'Tared over 200 samples. Offset F=[...] T=[...]'
```

- Keep the sensor **unloaded and still** during the tare.
- Only force/torque are offset; the IMU is untouched.
- The offset is held in the node and **resets on restart**, so tare after each launch.
- The service name matches Bota's official driver, so existing scripts keep working.
- The tare blocks ROS publishing for ~0.2 s. The EtherCAT loop keeps running.

---

## Shell aliases

Add these to your `~/.bashrc` and adjust the two paths to your install
(the defaults below are for the mc-rtc-superbuild):

```bash
# ── ethercat_publisher (ADC + Bota) ──────────────────────────────────────────
export ADC_BOTA_WS=~/workspace/src/catkin_data_ws/install          # colcon: ~/ros2_ws/install
export ADC_BOTA_NODE=$ADC_BOTA_WS/lib/ethercat_publisher/ethercat_publisher_node
#                    colcon: $ADC_BOTA_WS/ethercat_publisher/lib/ethercat_publisher/ethercat_publisher_node

# Grant capabilities (run after every build) and show the result
alias init_adc_bota='sudo setcap cap_net_raw,cap_net_admin,cap_sys_nice=+ep "$ADC_BOTA_NODE" && getcap "$ADC_BOTA_NODE"'

# Start the EtherCAT node
alias run_adc_bota='source "$ADC_BOTA_WS/setup.bash" && ros2 launch ethercat_publisher ethercat_publisher.launch.py'

# Tare the Bota force/torque sensor (node must be running)
alias calibrate_bota='source "$ADC_BOTA_WS/setup.bash" && ros2 service call /bota_ft_sensor/tare std_srvs/srv/Trigger {}'
```

Typical session:

```bash
init_adc_bota     # once after each build
run_adc_bota      # terminal 1
calibrate_bota    # terminal 2, sensor unloaded
```

Arguments still work with the alias, for example `run_adc_bota nic:=enx00e04c68027b`.

---

## Parameters

All parameters are also launch arguments:

```bash
ros2 launch ethercat_publisher ethercat_publisher.launch.py nic:=enx00e04c68027b
```

| Parameter | Default | Description |
|---|---|---|
| `nic` | `enp45s0` | Network interface connected to the EK1100 |
| `wrench_topic` | `/bus0/ft_sensor0/ft_sensor_readings/wrench` | WrenchStamped topic |
| `imu_topic` | `/bus0/ft_sensor0/ft_sensor_readings/imu` | Imu topic |
| `voltage_topic` | `/collision/voltage` | EL3102 voltage topic |
| `wrench_frame` | `FT_sensor_wrench` | `frame_id` of the WrenchStamped messages |
| `imu_frame` | `FT_sensor_imu` | `frame_id` of the Imu messages |
| `tare_service` | `/bota_ft_sensor/tare` | Name of the tare service |
| `tare_samples` | `200` | Number of samples averaged by the tare |
| `bota_sinc_length` | `51` | Bota sinc filter length written at startup: `51` = 1 kHz (`-1` = keep sensor value) |
| `bota_fir_disable` | `-1` | `1` = FIR off, `0` = FIR on |
| `bota_fast_enable` | `-1` | `1` = FAST spike filter on |
| `bota_chop_enable` | `-1` | `1` = CHOP on |
| `report_update_rate` | `true` | Log the measured loop rate and new-sample rate every 5 s |

---

## Bota sensor configuration

The publishing rate (1 kHz) is set by a ROS timer and does **not** prove that
the sensor produces 1000 new samples per second. At startup the node reads the
Bota configuration over SDO and logs it:

```
Bota config: sinc length=64, FIR disable=1, FAST=0, CHOP=0 | calibration=1, temp. comp.=0, IMU=1 | sampling rate=800 Hz
```

Every 5 s it also logs the **measured** rate of new force/torque samples
(frames whose values changed):

```
EtherCAT loop 1000 Hz | new Bota F/T samples 800 Hz
```

The force/torque output rate depends on the sinc length. Measured on a
`BFT-SENS-ECAT-M8`, it follows **rate ≈ 51 200 / sinc length**:

| Sinc length | F/T output rate |
|---|---|
| 51 | ≈ 1000 Hz |
| 64 | 800 Hz |
| 128 | 400 Hz |
| 205 | 250 Hz |
| 256 | 200 Hz |
| 512 | 100 Hz |

> Bota's user manual also contains a table listing 1000 Hz for sinc 51, 64
> and 128. That table is wrong for this sensor: trust the `sampling rate`
> read back from object `0x8011` and the measured `new Bota F/T samples` rate.

The IMU is a separate chip and is not affected by these settings.

By default the node writes **sinc length 51** (1 kHz). A shorter filter means
more bandwidth and more noise. If the sensor reports less than 1000 Hz, the
node logs a warning. To change it:

```bash
ros2 launch ethercat_publisher ethercat_publisher.launch.py bota_sinc_length:=64   # 800 Hz, less noise
ros2 launch ethercat_publisher ethercat_publisher.launch.py bota_sinc_length:=-1   # keep the sensor's stored setting
```

`0x8006:01` may read back as a different number than the one written (for
example 768 after writing 128). The sampling rate is the value that matters.

The settings are written in PRE-OP on every start and are **not** saved to the
sensor's flash, so a power cycle restores the stored configuration.

Relevant objects: `0x8006:01-04` (sinc, FIR, FAST, CHOP), `0x8010:01-03`
(calibration, temperature compensation, IMU), `0x8011` (sampling rate).

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `ecx_init failed on interface 'X'` | Capabilities lost after a rebuild | `init_adc_bota`, check `getcap` |
| | Interface name wrong / adapter unplugged / link down | `ip -br link`, `sudo ip link set X up`, set `nic:=...` |
| `No EtherCAT slaves found` | Wrong port (e.g. robot/LAN port with an IP) | Use the port cabled to the EK1100 |
| | EK1100 unpowered, or cable in X2 instead of X1 | Check 24 V (Us + Up) and the RJ45 port |
| `Expected 3 slaves ..., found N` | A device in the chain is unpowered or disconnected | Check the Bota supply and cabling |
| `Could not set SCHED_FIFO` | `cap_sys_nice` missing | `init_adc_bota` |
| Tare service waits forever | Node not running, or a different `tare_service` name | Start the node, check `ros2 service list` |
| `Tare timed out` | No valid frames from the sensor | Check the node logs for lost frames |

To see whether EtherCAT frames come back from the bus:

```bash
sudo tcpdump -i <iface> -e ether proto 0x88a4
```

Each frame should appear twice (sent and returned). Frames that appear only
once mean nothing on the bus is answering.

---

## Implementation notes

- Do not run `bota_driver_node` or any other EtherCAT master on the same bus at the same time.
- The cyclic loop runs at 1 kHz using `clock_nanosleep` (absolute time) with `SCHED_FIFO` and `mlockall`.
- ROS publishing is decoupled from the real-time thread: the loop caches the latest messages and a 1 ms ROS timer publishes them.
- If the working counter stays too low, the node automatically tries to recover the slaves.
- **EL3102**: decoded manually from the raw input bytes (compact PDO mapping),
  `voltage = (raw_int16 / 32767.0) * 10.0`.
- **Bota SensONE**: the PDO layout (`BotaPDO` in `include/ethercat_publisher.h`)
  was reverse-engineered from raw EtherCAT traffic. Acceleration is converted
  from g to m/s².
