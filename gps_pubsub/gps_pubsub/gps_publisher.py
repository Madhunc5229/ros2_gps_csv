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
        self.data_rows = []
        

        with open(csv_file_path, 'r') as csv_file:
            reader = csv.DictReader(csv_file)
            self.data_rows = list(reader)
        
            # for i,row in enumerate(reader):
            #     self.data_rows[i] = [row['Longitude'], row['Latitude'], row['Altitude'], row['Time'], row['Actual_Speed']]
            #     self.data_length = i+1
        self.i = 0
        self.timer = self.create_timer(1.0/3.0, self.publish_data)

    def publish_data(self):
        if(self.i<len(self.data_rows)):
            msg = Data()
            msg_data = self.data_rows[self.i]
            msg.longitude = float(msg_data['Longitude'])
            msg.latitude = float(msg_data['Latitude'])
            msg.altitude = float(msg_data['Altitude'])
            msg.time = msg_data['Time']
            msg.actual_speed = float(msg_data['Actual_Speed'])

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