"""
* @file gps_publisher.py
* @author Madhu Narra Chittibabu (madhunc117@gmail.com)
* @brief ROS2 Node which reads gps data from csv file and publishes it to 'data'
* @version 0.1
* @date 2023-07-16
* 
* @copyright Copyright (c) 2023

"""
import rclpy
from rclpy.node import Node
import csv
import os
from ament_index_python.packages import get_package_share_directory
from gps_msg_interface.msg import Data


class GpsPublsiher(Node):
    """GPS data publisher node class

    Args:
        Node (rclpy.node): Inheriting Node class
    """

    def __init__(self):
        """constructor of node class"""
        super().__init__("gps_publisher")
        self.publisher_ = self.create_publisher(Data, "data", 10)
        csv_file_path = os.path.join(
            get_package_share_directory("gps_pubsub"), "gps_data.csv"
        )
        # store the csv data
        self.data_rows = []

        with open(csv_file_path, "r") as csv_file:
            reader = csv.DictReader(csv_file)
            self.data_rows = list(reader)

        self.i = 0
        self.timer = self.create_timer(1.0 / 3.0, self.publish_data)

    def publish_data(self):
        """function that is called to publish msg to 'data' topic"""
        # check if the length greater than current element number
        if self.i < len(self.data_rows):
            msg = Data()
            msg_data = self.data_rows[self.i]
            msg.longitude = float(msg_data["Longitude"])
            msg.latitude = float(msg_data["Latitude"])
            msg.altitude = float(msg_data["Altitude"])
            msg.time = msg_data["Time"]
            msg.actual_speed = float(msg_data["Actual_Speed"])

            self.publisher_.publish(msg)
            self.i += 1
            log_msg = "MSG: {:.2f},  {:.2f},  {:.2f}, {}, {:.2f}".format(
                msg.longitude, msg.latitude, msg.altitude, msg.time, msg.actual_speed
            )
            self.get_logger().info(log_msg)

        else:
            self.get_logger().warn("Finished publishing the entire data")


def main(args=None):
    """main function to start the node

    Args:
        args (_type_, optional): Defaults to None.
    """
    rclpy.init(args=args)

    publisher_node = GpsPublsiher()

    rclpy.spin(publisher_node)

    publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
