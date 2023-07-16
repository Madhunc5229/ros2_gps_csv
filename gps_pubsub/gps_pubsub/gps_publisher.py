import rclpy
from rclpy.node import Node
import csv
import os
from ament_index_python.packages import get_package_share_directory
from gps_msg_interface.msg import Data



class GpsPublsiher(Node):

    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(Data, 'data', 10)
        csv_file_path = os.path.join(
            get_package_share_directory('gps_pubsub'),'gps_data.csv'
            )
        self.data_rows = {}
        self.data_length = 0

        with open(csv_file_path, 'r') as csv_file:
            reader = csv.DictReader(csv_file)
            for i,row in enumerate(reader):
                self.data_rows[i] = [row['Longitude'], row['Latitude'], row['Altitude'], row['Time'], row['Actual_Speed']]
                self.data_length = i+1
        self.i = 0
        self.timer = self.create_timer(1.0/3.0, self.publish_data)

    def publish_data(self):
        if(self.i<self.data_length):
            msg = Data()
            msg_data = self.data_rows[self.i]
            msg.longitude = float(msg_data[0])
            msg.latitude = float(msg_data[1])
            msg.altitude = float(msg_data[2])
            msg.time = msg_data[3]
            msg.actual_speed = float(msg_data[4])

            self.publisher_.publish(msg)
            self.i += 1
            log_msg = "MSG: {:.2f},  {:.2f},  {:.2f}, {}, {:.2f}".format( msg.longitude, msg.latitude, msg.altitude, msg.time, msg.actual_speed)
            self.get_logger().info(log_msg)

        else:
            self.get_logger().warn("Finished publishing the entire data")


def main(args=None):
    rclpy.init(args=args)

    publisher_node = GpsPublsiher()
    
    rclpy.spin(publisher_node)

    publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()