# Laser Scan Merger ROS Package
## Overview
The `laser_scan_merger` package allows you to merge multiple LaserScan messages from different LiDAR sensors into a single LaserScan or PointCloud message. This is useful for applications where data from multiple sensors need to be aggregated for use in mapping, navigation, or object detection.

## Features

- Merges multiple `sensor_msgs/LaserScan` topics into one.
- Converts merged `sensor_msgs/LaserScan` into `sensor_msgs/PointCloud` (optional).
- Configurable through a YAML configuration file.
- Supports transformation of LiDAR frames using a 6DOF transformation (translation + rotation).

## Prerequisites
### ROS (Robot Operating System) 
Ensure that you have a working ROS installation. You can install ROS by following the instructions at: [ROS Installation Guide][ROS Installation Guide]

[ROS Installation Guide]: https://wiki.ros.org/ROS/Installation

### YAML-cpp
This package depends on `yaml-cpp` for parsing the configuration files in YAML format.

To install `yaml-cpp`:
```shell
$ sudo apt-get install libyaml-cpp-dev
```

### PCL (Point Cloud Library)
If you wish to use the PointCloud merging feature, the package requires the `Point Cloud Library (PCL)`.

To install `PCL`:
```shell
$ sudo apt-get install libpcl-dev
```

## Installation
1. Clone the repository into your catkin workspace:
    ```shell
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/mjlee111/laser_scan_merger.git
    ```
2. Install dependencies using `rosdep`:
    ```shell
    $ rosdep install --from-paths src --ignore-src -r -y
    ```
3. Build the workspace:
    ```shell
    $ cd ~/catkin_ws
    $ catkin_make
    ```
4. Source your workspace:
    ```shell
    $ source devel/setup.bash
    ```

## Usage
You can start the `laser_scan_merger` node using the following command:
```shell
$ rosrun laser_scan_merger laser_scan_merger_node
```
Ensure that you have configured the `config/laser_config.yaml` file to define the topics, frames, and transformations for each of your LiDAR sensors.

### Example Configuration (YAML)
```yaml
merger:
  mergeFrameID: "/base_link"
  mergeLaserScanTopic: "/laserscan/merged"
  mergeLaserScanNum: 2
  publishPointCloud: true
  pointCloudTopic: "/laserscan/merged/points"

lidar1:
  laserScanTopic: "/laserscan1"
  laserScanframeID: "/laserscan1"
  tf: "1.0 0.0 0.2 0.0 0.0 0.0"

lidar2:
  laserScanTopic: "/laserscan2"
  laserScanframeID: "/laserscan2"
  tf: "2.0 0.0 0.2 0.0 0.0 0.0"
```
### YAML Configuration Table
| Parameter | Type | Description | Example Value |
|---|---|---|---|
| merger.mergeFrameID | string | The frame ID for the merged output. | "/base_link" |
| merger.mergeLaserScanTopic | string | The topic name where the merged LaserScan data will be published. | "/laserscan/merged" |
| merger.mergeLaserScanNum | int | The number of LaserScan topics to merge. | 2 |
| merger.publishPointCloud | bool | A flag to enable or disable PointCloud publishing. | true |
| merger.pointCloudTopic | string | The topic name where the merged PointCloud data will be published. This is required if PointCloud is enabled. | "/laserscan/merged/points" |
| lidarX.laserScanTopic | string | The ROS topic name for the LaserScan data from LiDAR sensor X. | "/laserscan1" |
| lidarX.laserScanframeID | string | The frame ID for the LaserScan data from LiDAR sensor X. | "/laserscan1" |
| lidarX.tf | string | A space-separated string of 6 float values representing the translation (x, y, z) and rotation (r, p, y) of the LiDAR sensor X. | "1.0 0.0 0.2 0.0 0.0 0.0" |

## Configuration
The package is configured using a YAML file that specifies the number of LiDAR sensors, their topics, and their respective transformation frames.

Make sure that your configuration file follows the structure in the example above, and define all the necessary LiDAR sensors.

## Troubleshooting
If you encounter any issues, ensure that the laser_config.yaml file is correctly formatted and contains all the required fields. Also, verify that the LiDAR topics are publishing correctly. 

## Contributing
Feel free to submit issues, feature requests, and pull requests to improve the package.

## License
This project is licensed under the MIT License - see the [LICENSE][LICENSE] file for details.

[LICENSE]: https://github.com/mjlee111/laser_scan_merger/blob/master/LICENSE
