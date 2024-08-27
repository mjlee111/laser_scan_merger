#include "../include/laser_scan_merger/laser_scan_merger.hpp"

LaserScanMerger::LaserScanMerger()
{
  nh_ = ros::NodeHandle("~");

  std::string package_path = ros::package::getPath("laser_scan_merger");
  std::string config_path = package_path + "/config/laser_config.yaml";

  ros::param::get("/laser_scan_merger/config_file_path", config_path);

  ROS_INFO_NAMED("Merger", "Found Config File on %s", config_path.c_str());
  loadConfigData(config_path);
  for (int i = 0; i < max_lidar_count; i++)
  {
    laser_scan_sub.push_back(nh_.subscribe<sensor_msgs::LaserScan>(
        laser_scan_topic[i], 1, boost::bind(&LaserScanMerger::LaserScanCallback, this, _1, i)));
  }
}

void LaserScanMerger::LaserScanCallback(const sensor_msgs::LaserScanConstPtr& laser, int num)
{
  scan_data[num] = *laser;
}

sensor_msgs::PointCloud LaserScanMerger::LaserScanToPointCloud(const sensor_msgs::LaserScanConstPtr& laser)
{
  sensor_msgs::PointCloud cloud;

  cloud.header = laser->header;

  cloud.points.resize(laser->ranges.size());

  for (size_t i = 0; i < laser->ranges.size(); ++i)
  {
    if (std::isfinite(laser->ranges[i]))
    {
      float angle = laser->angle_min + i * laser->angle_increment;
      float range = laser->ranges[i];

      cloud.points[i].x = range * std::cos(angle);
      cloud.points[i].y = range * std::sin(angle);
      cloud.points[i].z = 0.0;
    }
  }

  return cloud;
}

std::vector<float> LaserScanMerger::parseTfString(const std::string& tf_string)
{
  std::vector<float> tf_values;
  std::istringstream iss(tf_string);
  float value;

  while (iss >> value)
  {
    tf_values.push_back(value);
  }

  return tf_values;
}

int LaserScanMerger::loadConfigData(const std::string& path)
{
  std::ifstream file(path);
  if (!file.is_open())
  {
    ROS_ERROR("Configuration file '%s' does not exist or cannot be opened.", path.c_str());
    return -1;
  }

  YAML::Node config;
  try
  {
    config = YAML::LoadFile(path);
  }
  catch (const YAML::Exception& e)
  {
    ROS_ERROR("Failed to load configuration file '%s': %s", path.c_str(), e.what());
    return -1;
  }

  // Load merger configuration
  merge_frame_id = config["merger"]["mergeFrameID"].as<std::string>();
  merge_laser_scan_topic = config["merger"]["mergeLaserScanTopic"].as<std::string>();
  max_lidar_count = config["merger"]["mergeLaserScanNum"].as<int>();
  pointCloudPublish = config["merger"]["publishPointCloud"].as<bool>();

  ROS_INFO_NAMED("Merger", "Merged Laser Scan Frame ID: %s", merge_frame_id.c_str());
  ROS_INFO_NAMED("Merger", "Merged Laser Scan Topic: %s", merge_laser_scan_topic.c_str());
  ROS_INFO_NAMED("Merger", "Merging %d laser scan topics", max_lidar_count);

  if (pointCloudPublish)
  {
    pointCloudTopic = config["merger"]["pointCloudTopic"].as<std::string>();
    ROS_INFO_NAMED("Merger", "PointCloud publish enabled");
    ROS_INFO_NAMED("Merger", "Merged PointCloud Topic: %s", pointCloudTopic.c_str());
  }
  else
  {
    ROS_INFO_NAMED("Merger", "PointCloud publish disabled");
  }

  std::cout << std::endl;

  laser_scan_frame_id.resize(max_lidar_count);
  laser_scan_topic.resize(max_lidar_count);
  tf.resize(max_lidar_count);
  laser_scan_sub.resize(max_lidar_count);
  scan_data.resize(max_lidar_count);

  int actual_lidar_count = 0;
  for (int i = 1; i <= max_lidar_count; i++)
  {
    std::string name = "LiDAR" + std::to_string(i);
    ROS_INFO_NAMED(name.c_str(), "LiDAR Configuration %d", i);

    std::string lidar_key = "lidar" + std::to_string(i);
    if (config[lidar_key])
    {
      actual_lidar_count++;

      YAML::Node lidar = config[lidar_key];

      if (lidar["laserScanTopic"])
      {
        laser_scan_topic[i - 1] = lidar["laserScanTopic"].as<std::string>();
        ROS_INFO_NAMED(name.c_str(), "Laserscan Topic %d: %s", i, laser_scan_topic[i - 1].c_str());
      }
      else
      {
        ROS_WARN_NAMED(name.c_str(), "Missing 'laserScanTopic' in configuration for lidar %d", i);
      }

      if (lidar["laserScanframeID"])
      {
        laser_scan_frame_id[i - 1] = lidar["laserScanframeID"].as<std::string>();
        ROS_INFO_NAMED(name.c_str(), "Laserscan Frame ID %d: %s", i, laser_scan_frame_id[i - 1].c_str());
      }
      else
      {
        ROS_WARN_NAMED(name.c_str(), "Missing 'laserScanframeID' in configuration for lidar %d", i);
      }

      std::string tf_string = lidar["tf"].as<std::string>();
      std::vector<float> tf_values = parseTfString(tf_string);

      if (tf_values.size() != 6)
      {
        ROS_ERROR_NAMED(name.c_str(), "Error: 'tf' does not contain exactly 6 values on lidar %d", i);
        return -1;
      }

      std::array<float, 6> tf_;
      std::copy(tf_values.begin(), tf_values.end(), tf_.begin());
      tf[i - 1] = tf_;

      ROS_INFO_NAMED(name.c_str(), "Laserscan TF %d: %f %f %f %f %f %f", i, tf_[0], tf_[1], tf_[2], tf_[3], tf_[4],
                     tf_[5]);
    }
    else
    {
      ROS_ERROR_NAMED("Merger", "Configuration for %s not found. Only %d LiDAR configurations provided.",
                      lidar_key.c_str(), actual_lidar_count);
      throw std::runtime_error("Configuration for " + lidar_key + " not found. Insufficient LiDAR configurations.");
    }
  }

  return 1;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_scan_merger");
  LaserScanMerger laser_scan_merger;
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
