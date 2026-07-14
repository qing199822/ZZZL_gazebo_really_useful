// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ign_sim_pointcloud_tool/point_cloud_converter.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace ign_sim_pointcloud_tool
{
PointCloudConverter::PointCloudConverter(const rclcpp::NodeOptions & options)
: rclcpp::Node("point_cloud_converter", options)
{
  this->declare_parameter<std::string>("pcd_topic", "livox/lidar");
  this->declare_parameter<int>("n_scan", 32);
  this->declare_parameter<int>("horizon_scan", 1875);
  this->declare_parameter<float>("ang_bottom", 7.0);
  this->declare_parameter<float>("ang_res_y", 1.0);
  this->declare_parameter<float>("scan_period", 0.1f);  // H5: LiDAR scan period in seconds

  pcd_topic_ = this->get_parameter("pcd_topic").as_string();
  n_scan_ = this->get_parameter("n_scan").as_int();
  horizon_scan_ = this->get_parameter("horizon_scan").as_int();
  ang_bottom_ = static_cast<float>(this->get_parameter("ang_bottom").as_double());
  ang_res_y_ = static_cast<float>(this->get_parameter("ang_res_y").as_double());
  scan_period_ = this->get_parameter("scan_period").as_double();  // H5

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pcd_topic_, 10, std::bind(&PointCloudConverter::lidarHandle, this, std::placeholders::_1));

  pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "velodyne_points", rclcpp::SensorDataQoS());

  RCLCPP_INFO(this->get_logger(), "Listening to lidar topic: %s", pcd_topic_.c_str());
}

void PointCloudConverter::lidarHandle(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr original_pc(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<PointXYZIRT>::Ptr converted_pc(new pcl::PointCloud<PointXYZIRT>());

  try {
    pcl::fromROSMsg(*pc_msg, *original_pc);
  } catch (const pcl::InvalidConversionException & e) {
    RCLCPP_ERROR(this->get_logger(),
      "PointCloud conversion failed: %s. Message may lack XYZ fields.", e.what());
    return;
  }

  // H7: track drop rate for warnings
  size_t total_input = original_pc->points.size();
  size_t dropped_count = 0;
  static size_t cumulative_dropped = 0, cumulative_total = 0;

  for (size_t point_id = 0; point_id < total_input; ++point_id) {
    PointXYZIRT new_point;
    const auto & old_point = original_pc->points[point_id];

    new_point.x = old_point.x;
    new_point.y = old_point.y;
    new_point.z = old_point.z;
    // H6: intensity is 0 for PointXYZ input (Gazebo LiDAR outputs XYZ only)
    // If source type gains intensity in future, set it here
    new_point.intensity = 0;

    float vertical_angle =
      std::atan2(new_point.z, std::sqrt(new_point.x * new_point.x + new_point.y * new_point.y)) *
      180 / M_PI;
    int row_id = static_cast<int>((vertical_angle + ang_bottom_) / ang_res_y_);

    if (row_id >= 0 && row_id < n_scan_) {
      new_point.ring = row_id;
    } else {
      ++dropped_count;
      continue;
    }

    // H5: use configurable scan_period instead of hardcoded 0.1f
    new_point.time = (point_id % horizon_scan_) * scan_period_ / horizon_scan_;

    converted_pc->points.push_back(new_point);
  }

  // H7: warn when point drop rate is excessive
  cumulative_dropped += dropped_count;
  cumulative_total += total_input;
  if (cumulative_total > 50000 && cumulative_dropped > cumulative_total / 4) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
      "%.1f%% of points dropped (%zu/%zu). Check ang_bottom=%.1f ang_res_y=%.2f n_scan=%d",
      100.0 * cumulative_dropped / cumulative_total, cumulative_dropped, cumulative_total,
      ang_bottom_, ang_res_y_, n_scan_);
    cumulative_dropped = 0;
    cumulative_total = 0;
  }

  publishPoints<PointXYZIRT>(converted_pc, *pc_msg);
}

template <typename T>
void PointCloudConverter::publishPoints(
  const typename pcl::PointCloud<T>::Ptr & new_pc, const sensor_msgs::msg::PointCloud2 & old_msg)
{
  new_pc->is_dense = true;  // L4: all remaining points are valid finite points

  sensor_msgs::msg::PointCloud2 pc_new_msg;
  pcl::toROSMsg(*new_pc, pc_new_msg);
  pc_new_msg.header = old_msg.header;

  pcd_pub_->publish(pc_new_msg);
}

}  // namespace ign_sim_pointcloud_tool

// M4: explicit template instantiation to allow use from other translation units
template void ign_sim_pointcloud_tool::PointCloudConverter::publishPoints<PointXYZIRT>(
  const pcl::PointCloud<PointXYZIRT>::Ptr &, const sensor_msgs::msg::PointCloud2 &);

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ign_sim_pointcloud_tool::PointCloudConverter)
