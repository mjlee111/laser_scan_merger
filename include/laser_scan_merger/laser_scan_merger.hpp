#ifndef LASER_SCAN_MERGER_HPP
#define LASER_SCAN_MERGER_HPP

#include <iostream>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <string>
#include <array>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Matrix3x3.h>

class LaserScanMerger
{
public:
  LaserScanMerger();
  void LaserScanCallback(const sensor_msgs::LaserScanConstPtr& laser, int num);
  sensor_msgs::PointCloud LaserScanToPointCloud(const sensor_msgs::LaserScan& laser);
  void transformLaserScan(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& output_scan,
                          const std::array<float, 6>& tf);
  void PublishMergedLaserScan();
  bool AllScansReceived();

private:
  ros::NodeHandle nh_;

  std::vector<float> parseTfString(const std::string& tf_string);
  int loadConfigData(const std::string& path);

  int max_lidar_count;
  std::string merge_frame_id;
  std::string merge_laser_scan_topic;
  bool point_cloud_publish;
  std::string point_cloud_topic;
  float timeout = 1.0;  // [s]

  std::vector<ros::Subscriber> laser_scan_sub;
  std::vector<std::string> laser_scan_topic;
  std::vector<std::string> laser_scan_frame_id;
  std::vector<std::array<float, 6>> tf;
  std::vector<sensor_msgs::LaserScan> scan_data;
  std::vector<ros::Time> scan_time;

  ros::Publisher merged_scan_pub;
  ros::Publisher merged_points_pub;
};

#endif
