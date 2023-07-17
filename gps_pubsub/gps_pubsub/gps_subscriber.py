"""
* @file gps_subscriber.py
* @author Madhu Narra Chittibabu (madhunc117@gmail.com)
* @brief ROS2 Node which subscribes to 'data' and publishes time diff to 'diff'
* @version 0.1
* @date 2023-07-16
* 
* @copyright Copyright (c) 2023

"""
import rclpy
from rclpy.node import Node
from gps_msg_interface.msg import Data
from std_msgs.msg import Float32
import time


class GpsSubscriber(Node):
    """GPS data subscriber node class

    Args:
        Node (rclpy.node): Inheriting Node class
    """

    def __init__(self):
        """constructor of node class"""
        super().__init__("gps_data_subscriber")
        self.subscriber_ = self.create_subscription(Data, "data", self.process_data, 10)
        self.publisher_ = self.create_publisher(Float32, "diff", 10)
        self.previous_time = None

    def process_data(self, msg):
        """callback function to process msg from 'data'

        Args:
            msg (Data): GPS data
        """
        # convert time from str to time struct
        t1 = time.strptime(msg.time, "%Y-%m-%d %H:%M:%S+00:00")
        if self.previous_time is not None:
            # convert time struct to time
            current_time = time.mktime(t1)
            time_diff = current_time - self.previous_time
            diff_msg = Float32()
            diff_msg.data = time_diff
            self.publisher_.publish(diff_msg)
            self.get_logger().info('Time Difference: "%f"' % time_diff)
        self.previous_time = time.mktime(t1)


def main(args=None):
    """main function to start the node

    Args:
        args (_type_, optional): Defaults to None.
    """
    rclpy.init(args=args)

    subscriber_node = GpsSubscriber()

    rclpy.spin(subscriber_node)

    subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
