from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    sensor_filter_node = Node(package="sensor_filter_kit",
                              executable="sensor_filter_kit_node",
                              parameters=[{"/imu_raw_topic":"/imu0"},
                                          {"/imu_filtered_topic":"/imu/filtered"},
                                          {"/window_size":10}]
                             )

    ld.add_action(sensor_filter_node)

    return ld