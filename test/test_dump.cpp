// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <fstream>

#include "dmabuf_transport/type/image.hpp"
#include "dmabuf_transport/type/point_cloud2.hpp"
#include "pcl/filters/filter.h"
#include "pcl/io/ply_io.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"

namespace dmabuf_transport
{

class TestDumpComponent : public rclcpp::Node
{
public:
  explicit TestDumpComponent(const rclcpp::NodeOptions & options);

private:
  rclcpp::SubscriptionBase::SharedPtr sub_{ nullptr };

  template <typename T>
  void create_dump_subscriber();

  void dump_msg(const std::shared_ptr<dmabuf_transport::type::Image> msg);
  void dump_msg(const std::shared_ptr<sensor_msgs::msg::Image> msg);
  void dump_msg(const std::shared_ptr<dmabuf_transport::type::PointCloud2> msg);
  void dump_msg(const std::shared_ptr<sensor_msgs::msg::PointCloud2> msg);

  void save_data_to_file(const std::string & path, const char * data, std::size_t size);
  void read_dmabuf_to_file(const std::string & path,
      std::shared_ptr<lib_mem_dmabuf::DmaBuffer> dmabuf);

  std::string dump_path_;
  std::string dump_topic_;
};

TestDumpComponent::TestDumpComponent(const rclcpp::NodeOptions & options)
  : Node("TestDumpComponent", options)
{
  auto message_type = this->declare_parameter("test_type", "dmabuf_transport::type::Image");
  dump_topic_ = this->declare_parameter("topic_name", "image");
  dump_path_ = this->declare_parameter("dump_file", "/data/dump");

  RCLCPP_INFO(
      get_logger(), "=== Dump type: %s, topic_name: %s", message_type.c_str(), dump_topic_.c_str());

  if (message_type == "dmabuf_transport::type::Image") {
    create_dump_subscriber<dmabuf_transport::type::Image>();
  } else if (message_type == "dmabuf_transport::type::PointCloud2") {
    create_dump_subscriber<dmabuf_transport::type::PointCloud2>();
  } else if (message_type == "sensor_msgs::msg::Image") {
    create_dump_subscriber<sensor_msgs::msg::Image>();
  } else if (message_type == "sensor_msgs::msg::PointCloud2") {
    create_dump_subscriber<sensor_msgs::msg::PointCloud2>();
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "Unknown type: " << message_type);
  }
}

template <typename T>
void TestDumpComponent::create_dump_subscriber()
{
  sub_ = this->create_subscription<T>(
      dump_topic_, 30, [this](const std::shared_ptr<T> msg) { dump_msg(msg); });
}

void TestDumpComponent::save_data_to_file(const std::string & path,
    const char * data,
    std::size_t size)
{
  std::ofstream file(path, std::ios::binary);
  if (!file.is_open()) {
    RCLCPP_ERROR_STREAM(get_logger(), "open file: " << path << "failed");
    return;
  }

  file.write(data, size);
  file.close();
}

void TestDumpComponent::read_dmabuf_to_file(const std::string & path,
    std::shared_ptr<lib_mem_dmabuf::DmaBuffer> dmabuf)
{
  if (dmabuf == nullptr) {
    RCLCPP_ERROR(get_logger(), "dmabuf is null");
    return;
  }

  if (!dmabuf->map() || !dmabuf->sync_start()) {
    RCLCPP_INFO(get_logger(), "read from dmabuf failed");
    return;
  }

  save_data_to_file(path, (char *)dmabuf->addr(), dmabuf->size());
  if (!dmabuf->sync_end() || !dmabuf->unmap()) {
    RCLCPP_INFO(get_logger(), "read from dmabuf failed");
    return;
  }
}

void TestDumpComponent::dump_msg(const std::shared_ptr<type::Image> msg)
{
  RCLCPP_INFO(get_logger(), "dump dmabuf_transport::type::Image message");
  read_dmabuf_to_file(dump_path_, msg->dmabuf);
  RCLCPP_INFO(get_logger(), "dump success");
  sub_.reset();
}

void TestDumpComponent::dump_msg(const std::shared_ptr<sensor_msgs::msg::Image> msg)
{
  RCLCPP_INFO(get_logger(), "dump sensor_msgs::msg::Image message");
  save_data_to_file(dump_path_, (char *)msg->data.data(), msg->data.size());
  RCLCPP_INFO(get_logger(), "dump success");
  sub_.reset();
}

void TestDumpComponent::dump_msg(const std::shared_ptr<type::PointCloud2> msg)
{
  RCLCPP_INFO(get_logger(), "dump dmabuf_transport::type::PointCloud2 message");
  auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  rclcpp::TypeAdapter<type::PointCloud2, sensor_msgs::msg::PointCloud2>::convert_to_ros_message(
      *msg, *ros_msg);
  dump_msg(ros_msg);
}

void TestDumpComponent::dump_msg(const std::shared_ptr<sensor_msgs::msg::PointCloud2> msg)
{
  RCLCPP_INFO(get_logger(), "dump sensor_msgs::msg::PointCloud2 message");

  pcl::PointCloud<pcl::PointXYZ> p;
  pcl::fromROSMsg(*msg, p);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(p, p, indices);

  if (pcl::io::savePLYFile(dump_path_, p) == -1) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "save to file: " << dump_path_ << "failed");
    sub_.reset();
    return;
  }

  RCLCPP_INFO(get_logger(), "dump success");
  sub_.reset();
}

}  // namespace dmabuf_transport

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dmabuf_transport::TestDumpComponent)
