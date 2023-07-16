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
from std_msgs.msg import Float32


#Decorator to run tests with colcon test
@pytest.mark.rostest
def generate_test_description():
    file_path = os.path.dirname(__file__)
    gps_publisher_node = launch_ros.actions.Node(
        executable = sys.executable,
        arguments=[os.path.join(
        file_path, "..", "gps_pubsub", "gps_publisher.py"
        )],
        additional_env = {'PYTHONUNBUFFERED':'1'},

    )
    gps_subscriber_node = launch_ros.actions.Node(
        executable = sys.executable,
        arguments=[os.path.join(
        file_path, "..", "gps_pubsub", "gps_subscriber.py"
        )],
        additional_env = {'PYTHONUNBUFFERED':'1'},

    )
    return (
        launch.LaunchDescription([
            gps_publisher_node,
            gps_subscriber_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {   
            'gps_publisher': gps_publisher_node,
            'gps_subscriber': gps_subscriber_node,
        }
    )

class TestGpsSubscriber(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def setUp(self):
        self.node = rclpy.create_node('test_gps_subscriber')
    
    def tearDown(self) -> None:
       self.node.destroy_node()
    
    def test_gps_publisher_data(self, gps_subscriber, proc_output):

        msgs_recieved = []
        
        csv_file_path = os.path.join(
            get_package_share_directory('gps_pubsub'),'gps_data.csv'
            )
        with open(csv_file_path, 'r') as csv_file:
            reader = csv.DictReader(csv_file)
            data = list(reader)
        i = 0
        msg = Data()

        try:
            
            sub = self.node.create_subscription(
            Float32,
            'diff',
            lambda msg : msgs_recieved.append(msg),
            10
            )
            end_time = time.time() + 5
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if len(msgs_recieved) > 2:
                    break
            self.assertGreater(len(msgs_recieved),1)

            for i,msg in enumerate(msgs_recieved):
                self.assertEqual(float(data[i+1]['Time_diff']), msg.data)
        finally:
            self.node.destroy_subscription(sub)
