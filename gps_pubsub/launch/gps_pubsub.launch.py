from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    gps_publisher_node = Node(
        package="gps_pubsub",
        executable="publish_gps_data",
    )

    gps_subscriber_node = Node(
        package="gps_pubsub",
        executable="subscribe_gps_data",
    )

    ld.add_action(gps_publisher_node)
    ld.add_action(gps_subscriber_node)

    return ld
