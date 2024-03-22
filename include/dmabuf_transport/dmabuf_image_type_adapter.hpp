// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef DMABUF_TRANSPORT__DMABUF_IMAGE_TYPE_ADAPTER
#define DMABUF_TRANSPORT__DMABUF_IMAGE_TYPE_ADAPTER

#include <cstring>
#include <memory>

#include "dmabuf_transport/dmabuf_type_adapter.hpp"
#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/msg/image.hpp"

template <>
struct rclcpp::TypeAdapter<dmabuf_transport::DmaBufTypeAdapter<sensor_msgs::msg::Image>,
    sensor_msgs::msg::Image>
{
  using is_specialized = std::true_type;
  using custom_type = dmabuf_transport::DmaBufTypeAdapter<sensor_msgs::msg::Image>;
  using ros_message_type = sensor_msgs::msg::Image;

  static void convert_to_ros_message(const custom_type& source, ros_message_type& destination)
  {
    // copy message informations, not include data
    destination = source.get_ros_message();
    // copy message data
    destination.data.resize(source.get_dma_buf()->size());
    std::memcpy(
        destination.data.data(), source.get_dma_buf()->addr(), source.get_dma_buf()->size());
  }

  static void convert_to_custom(const ros_message_type& source, custom_type& destination)
  {
    auto heap = "/dev/dma_heap/system";
    auto dmabuf = dmabuf_transport::DmaBuffer::alloc(source.data.size(), heap);
    if (dmabuf == nullptr) {
      return;
    }
    if (!dmabuf->set_data((void*)source.data.data(), source.data.size())) {
      return;
    }
    destination.set_dma_buf(dmabuf);
    destination.set_ros_message(source);
  }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
    dmabuf_transport::DmaBufTypeAdapter<sensor_msgs::msg::Image>,
    sensor_msgs::msg::Image);

#endif  // DMABUF_TRANSPORT__DMABUF_IMAGE_TYPE_ADAPTER