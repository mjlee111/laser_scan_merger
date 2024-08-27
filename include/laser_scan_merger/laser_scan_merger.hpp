#ifndef LASER_SCAN_MERGER_HPP
#define LASER_SCAN_MERGER_HPP

#include <iostream>
#include <sstream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <string>
#include <array>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>

class LaserScanMerger
{
public:
  LaserScanMerger();
  void LaserScanCallback(const sensor_msgs::LaserScanConstPtr& laser, int num);
  sensor_msgs::PointCloud LaserScanToPointCloud(const sensor_msgs::LaserScanConstPtr& laser);

private:
  ros::NodeHandle nh_;

  std::vector<float> parseTfString(const std::string& tf_string);
  int loadConfigData(const std::string& path);

  int max_lidar_count;
  std::string merge_frame_id;
  std::string merge_laser_scan_topic;
  bool pointCloudPublish;
  std::string pointCloudTopic;

  std::vector<ros::Subscriber> laser_scan_sub;
  std::vector<std::string> laser_scan_topic;
  std::vector<std::string> laser_scan_frame_id;
  std::vector<std::array<float, 6>> tf;
  std::vector<sensor_msgs::LaserScan> scan_data;

  ros::Publisher merged_scan_pub_;
};

#endif
