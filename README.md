[![Build](https://github.com/Madhunc5229/ros2_gps_csv/actions/workflows/main.yml/badge.svg?branch=main)](https://github.com/Madhunc5229/ros2_gps_csv/actions/workflows/main.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
# ros2_gps_csv
This is ROS2 software package which contains a `gps_msg_interface` and `gps_pubsub` ros2 packages. The `gps_msg_interface` package has implementation of custom msg Data which comprises of GPS attributes. The `gps_pubsub` has a publisher and subscriber node. The publisher node publishes GPS data from a CSV file to the `data` topic. The subscriber subscribes to the `data` topic and computes the time difference between time fields of two consecutive messages. 


 ## Dependencies
 - ROS2 Humble
 - Colcon
 - Python
 - Pytest
 - Cmake min 3.8 
 - Ubuntu 22.04



## Building the package
- Open a new terminal on your linux system and type the following commands 

#### source the ROS2 setup bash.
```
source /opt/ros/humble/setup.bash
```
#### Clone the repository
```
cd <ros2 workspace folder>/src
git clone https://github.com/Madhunc5229/ros2_gps_csv.git
```

#### Build and source the packages using colcon
```
colcon build && source install/setup.bash

```

## Running tests
- To run test cases, type the following command in the terminal
  ```
  colcon test --packages-select gps_pubsub --event-handlers console_direct+
  ```
## Running Nodes

#### Run the publisher node
```
ros2 run gps_pubsub publish_gps_data
```
#### Run the subscriber node
- Open a new terminal and source the bash file
    ```
    source install/setup.bash
    ```
- Run the subscriber
  ```
  ros2 run gps_pubsub subscribe_gps_data
  ```

## Launching Nodes (Optional) 
- If you want to launch both the nodes at the same, type the following in a new terminal
    ```
    source install/setup.bash
    ros2 launch gps_pubsub gps_pubsub.launch.py
    ```
## Build with coverage

- if you want to build with coverage and generate code coverage report:
  ```
    colcon test --packages-select gps_pubsub --pytest-with-coverage
  ```
