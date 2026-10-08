from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Launch arguments (name, default, description). Each one is forwarded as a node parameter.
ARGS = [
    # Network interface that owns the EtherCAT bus (dedicated port, nothing else on it)
    ("nic",           "enp45s0", "Network interface connected to the EK1100 (X1 / IN)"),
    # Topic names — must match RosForceSensor / RosImuSensor plugin config
    ("wrench_topic",  "/bus0/ft_sensor0/ft_sensor_readings/wrench", "WrenchStamped topic"),
    ("imu_topic",     "/bus0/ft_sensor0/ft_sensor_readings/imu",    "Imu topic"),
    ("voltage_topic", "/collision/voltage",                          "EL3102 voltage topic"),
    # Frame IDs — must match mc_rtc robot module sensor frames
    ("wrench_frame",  "FT_sensor_wrench", "frame_id of the WrenchStamped messages"),
    ("imu_frame",     "FT_sensor_imu",    "frame_id of the Imu messages"),
    # Software tare
    ("tare_service",  "/bota_ft_sensor/tare", "Name of the std_srvs/Trigger tare service"),
    ("tare_samples",  "200",                  "Samples averaged by the tare (1 kHz -> 200 = 0.2 s)"),
    # Bota filter, written over SDO at startup (default: 1 kHz). -1 = keep the value stored on the sensor
    ("bota_sinc_length", "51", "Sinc filter length, rate ~ 51200/sinc: 51 = 1 kHz, 64 = 800 Hz, 128 = 400 Hz; -1 = keep sensor value"),
    ("bota_fir_disable", "-1", "1 = FIR filter off, 0 = on"),
    ("bota_fast_enable", "-1", "1 = FAST (spike) filter on, 0 = off"),
    ("bota_chop_enable", "-1", "1 = CHOP on, 0 = off"),
    # Log the real EtherCAT loop rate and the rate of new F/T samples every 5 s
    ("report_update_rate", "true", "Log measured sensor update rate"),
]

INT_ARGS = ["tare_samples", "bota_sinc_length", "bota_fir_disable",
            "bota_fast_enable", "bota_chop_enable"]
BOOL_ARGS = ["report_update_rate"]


def generate_launch_description():
    params = {name: LaunchConfiguration(name) for name, _, _ in ARGS}
    for name in INT_ARGS:
        params[name] = ParameterValue(LaunchConfiguration(name), value_type=int)
    for name in BOOL_ARGS:
        params[name] = ParameterValue(LaunchConfiguration(name), value_type=bool)

    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=desc)
         for name, default, desc in ARGS]
        + [
            Node(
                package="ethercat_publisher",
                executable="ethercat_publisher_node",
                name="ethercat_publisher",
                output="screen",
                parameters=[params],
                # Give the node raw socket access without running as full root:
                # sudo setcap cap_net_admin,cap_net_raw,cap_sys_nice=+ep <install>/lib/ethercat_publisher/ethercat_publisher_node
                # NOTE: rebuilding/reinstalling the node erases these capabilities -> re-run setcap after every build
            )
        ]
    )
