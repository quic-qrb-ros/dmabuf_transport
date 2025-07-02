// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef DMABUF_TRANSPORT__TYPE__POINT_CLOUD2_HPP_
#define DMABUF_TRANSPORT__TYPE__POINT_CLOUD2_HPP_

#include "lib_mem_dmabuf/dmabuf.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace dmabuf_transport::type
{

struct PointCloud2
{
  std_msgs::msg::Header header;
  uint32_t width;
  uint32_t height;
  std::vector<sensor_msgs::msg::PointField> fields;
  bool is_bigendian;
  uint32_t point_step;
  uint32_t row_step;
  std::shared_ptr<lib_mem_dmabuf::DmaBuffer> dmabuf;
  bool is_dense;
};

}  // namespace dmabuf_transport::type

template <>
struct rclcpp::TypeAdapter<dmabuf_transport::type::PointCloud2, sensor_msgs::msg::PointCloud2>
{
  using is_specialized = std::true_type;
  using custom_type = dmabuf_transport::type::PointCloud2;
  using ros_message_type = sensor_msgs::msg::PointCloud2;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(dmabuf_transport::type::PointCloud2,
    sensor_msgs::msg::PointCloud2);

#endif  // DMABUF_TRANSPORT__TYPE__POINT_CLOUD2_HPP_
