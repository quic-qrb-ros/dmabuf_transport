// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef DMABUF_TRANSPORT__TYPE__IMAGE_HPP_
#define DMABUF_TRANSPORT__TYPE__IMAGE_HPP_

#include "lib_mem_dmabuf/dmabuf.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace dmabuf_transport::type
{

struct Image
{
  std_msgs::msg::Header header;
  uint32_t width;
  uint32_t height;
  std::string encoding;
  uint8_t is_bigendian;
  uint32_t step;
  std::shared_ptr<lib_mem_dmabuf::DmaBuffer> dmabuf;
};

}  // namespace dmabuf_transport::type

template <>
struct rclcpp::TypeAdapter<dmabuf_transport::type::Image, sensor_msgs::msg::Image>
{
  using is_specialized = std::true_type;
  using custom_type = dmabuf_transport::type::Image;
  using ros_message_type = sensor_msgs::msg::Image;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(dmabuf_transport::type::Image,
    sensor_msgs::msg::Image);

#endif  // DMABUF_TRANSPORT__TYPE__IMAGE_HPP_
