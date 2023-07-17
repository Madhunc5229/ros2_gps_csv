"""
* @file test_gps_publisher.py
* @author Madhu Narra Chittibabu (madhunc117@gmail.com)
* @brief test script to unittest gps_publisher node
* @version 0.1
* @date 2023-07-16
* 
* @copyright Copyright (c) 2023

"""
import os
import sys
import time
import unittest
import pytest
import csv
from ament_index_python.packages import get_package_share_directory


import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions

import rclpy

from gps_msg_interface.msg import Data


# Decorator to run tests with colcon test
@pytest.mark.rostest
def generate_test_description():
    """Generate ROS2 launch description"""
    file_path = os.path.dirname(__file__)
    gps_publisher_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(file_path, "..", "gps_pubsub", "gps_publisher.py")],
        additional_env={"PYTHONUNBUFFERED": "1"},
    )
    return (
        launch.LaunchDescription(
            [
                gps_publisher_node,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            "gps_publisher": gps_publisher_node,
        },
    )


class TestGpsPublisher(unittest.TestCase):
    """class to test the data published by gps_publisher node

    Args:
        unittest : Inherited from unittest.TestCase
    """

    @classmethod
    def setUpClass(self):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_gps_publisher")

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_gps_publisher_data(self):
        """run multiple tests on the data published"""
        msgs_recieved = []

        sub = self.node.create_subscription(
            Data, "data", lambda msg: msgs_recieved.append(msg), 10
        )
        csv_file_path = os.path.join(
            get_package_share_directory("gps_pubsub"), "gps_data.csv"
        )

        with open(csv_file_path, "r") as csv_file:
            reader = csv.DictReader(csv_file)
            data = list(reader)

        try:
            end_time = time.time() + 1
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if len(msgs_recieved) > 2:
                    break

            # test to check the number of messages
            self.assertGreater(len(msgs_recieved), 1)

            for i, msg in enumerate(msgs_recieved):
                logt, lat, alt, tim, speed = (
                    msg.longitude,
                    msg.latitude,
                    msg.altitude,
                    msg.time,
                    msg.actual_speed,
                )
                # Test the type of data published
                self.assertEqual(float, type(logt))
                self.assertEqual(float, type(lat))
                self.assertEqual(float, type(alt))
                self.assertEqual(str, type(tim))
                self.assertEqual(float, type(speed))

                # Test the value of data published
                self.assertEqual(float(data[i]["Longitude"]), logt)
                self.assertEqual(float(data[i]["Latitude"]), lat)
                self.assertEqual(float(data[i]["Altitude"]), alt)
                self.assertEqual(data[i]["Time"], tim)
                self.assertEqual(float(data[i]["Actual_Speed"]), speed)

        finally:
            self.node.destroy_subscription(sub)
