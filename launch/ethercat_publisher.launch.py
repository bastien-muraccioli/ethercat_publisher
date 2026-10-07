from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ethercat_publisher",
            executable="ethercat_publisher_node",
            name="ethercat_publisher",
            output="screen",
            parameters=[{
                # Network interface that owns the EtherCAT bus
                "nic": "enx00e04c68027b",

                # Topic names — must match RosForceSensor / RosImuSensor plugin config
                "wrench_topic":  "/bus0/ft_sensor0/ft_sensor_readings/wrench",
                "imu_topic":     "/bus0/ft_sensor0/ft_sensor_readings/imu",
                "voltage_topic": "/collision/voltage",

                # Frame IDs — must match mc_rtc robot module sensor frames
                "wrench_frame": "FT_sensor_wrench",
                "imu_frame":    "FT_sensor_imu",
            }],
            # Give the node raw socket access without running as full root:
            # sudo setcap cap_net_raw,cap_sys_nice=+ep install/lib/ethercat_publisher/ethercat_publisher_node
        )
    ])
