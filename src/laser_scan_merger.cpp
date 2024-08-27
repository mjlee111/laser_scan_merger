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
        laser_scan_topic[i], 10, boost::bind(&LaserScanMerger::LaserScanCallback, this, _1, i)));
  }
  merged_scan_pub = nh_.advertise<sensor_msgs::LaserScan>(merge_laser_scan_topic, 1);
}

void LaserScanMerger::LaserScanCallback(const sensor_msgs::LaserScanConstPtr& laser, int num)
{
  scan_time[num] = laser->header.stamp;
  transformLaserScan(*laser, scan_data[num], tf[num]);
}

bool LaserScanMerger::AllScansReceived()
{
  if (scan_time.empty())
  {
    return false;
  }

  ros::Time current_time = ros::Time::now();

  for (const auto& scan_time_point : scan_time)
  {
    ros::Duration duration_since_scan = current_time - scan_time_point;
    if (duration_since_scan.toSec() > timeout)
    {
      ROS_WARN("TIMEOUT");
      return false;
    }
  }

  return true;
}

void LaserScanMerger::PublishMergedLaserScan()
{
  if (AllScansReceived())
  {
    sensor_msgs::LaserScan merged_scan;
    merged_scan.header.frame_id = merge_frame_id;
    merged_scan.header.stamp = ros::Time::now();
    merged_scan.angle_min = 0.0;
    merged_scan.angle_max = 2 * M_PI;
    merged_scan.angle_increment = M_PI / 180.0;
    merged_scan.range_min = 0.0;
    merged_scan.range_max = 100.0;

    size_t num_ranges = 360;
    merged_scan.ranges.resize(num_ranges, std::numeric_limits<float>::infinity());
    merged_scan.intensities.resize(num_ranges, 0.0);

    for (const auto& scan : scan_data)
    {
      for (size_t i = 0; i < scan.ranges.size(); ++i)
      {
        if (scan.ranges[i] < merged_scan.ranges[i])
        {
          merged_scan.ranges[i] = scan.ranges[i];
          merged_scan.intensities[i] = scan.intensities[i];
        }
      }
    }

    merged_scan_pub.publish(merged_scan);
  }
}

void LaserScanMerger::transformLaserScan(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& output_scan,
                                         const std::array<float, 6>& tf)
{
  float x = tf[0];
  float y = tf[1];
  float z = tf[2];
  float roll = tf[3];
  float pitch = tf[4];
  float yaw = tf[5];

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(x, y, z));
  tf::Quaternion quaternion;
  quaternion.setRPY(roll, pitch, yaw);
  transform.setRotation(quaternion);

  output_scan = input_scan;
  output_scan.header.frame_id = "transformed_laser_scan";

  int num_points = input_scan.ranges.size();
  output_scan.ranges.resize(num_points);
  output_scan.intensities.resize(num_points);

  for (int i = 0; i < num_points; ++i)
  {
    float angle = input_scan.angle_min + i * input_scan.angle_increment;
    float range = input_scan.ranges[i];

    float x_old = range * cos(angle);
    float y_old = range * sin(angle);

    tf::Vector3 old_point(x_old, y_old, 0.0);
    tf::Vector3 new_point = transform * old_point;

    output_scan.ranges[i] = sqrt(new_point.x() * new_point.x() + new_point.y() * new_point.y());
  }
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

  merge_frame_id = config["merger"]["mergeFrameID"].as<std::string>();
  merge_laser_scan_topic = config["merger"]["mergeLaserScanTopic"].as<std::string>();
  max_lidar_count = config["merger"]["mergeLaserScanNum"].as<int>();
  timeout = config["merger"]["mergeTimeout"].as<float>();
  pointCloudPublish = config["merger"]["publishPointCloud"].as<bool>();

  ROS_INFO_NAMED("Merger", "Merged Laser Scan Frame ID: %s", merge_frame_id.c_str());
  ROS_INFO_NAMED("Merger", "Merged Laser Scan Topic: %s", merge_laser_scan_topic.c_str());
  ROS_INFO_NAMED("Merger", "Merging %d laser scan topics", max_lidar_count);
  ROS_INFO_NAMED("Merger", "Merger Timeout setted to : %f [s]", timeout);

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
  scan_time.resize(max_lidar_count);

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
