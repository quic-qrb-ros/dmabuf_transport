// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <fstream>

#include "dmabuf_transport/type/image.hpp"
#include "dmabuf_transport/type/point_cloud2.hpp"
#include "pcl/io/ply_io.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace dmabuf_transport
{

class TestPubComponent : public rclcpp::Node
{
public:
  explicit TestPubComponent(const rclcpp::NodeOptions & options);
  ~TestPubComponent();

private:
  std::unique_ptr<char[]> read_data_from_file(const std::string & path, std::size_t size);
  std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>> read_pcl_from_file(const std::string & path);

  void publish_image(int width,
      int height,
      const std::string & encoding,
      int fps,
      const std::string & data_file);

  void publish_ros_image(int width,
      int height,
      const std::string & encoding,
      int fps,
      const std::string & data_file);

  void publish_pcl(int fps, const std::string & data_file);
  void publish_ros_pcl(int fps, const std::string & data_file);

  rclcpp::Publisher<type::Image>::SharedPtr image_pub_{ nullptr };
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_image_pub_{ nullptr };
  rclcpp::Publisher<type::PointCloud2>::SharedPtr pcl_pub_{ nullptr };
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ros_pcl_pub_{ nullptr };

  std::shared_ptr<std::thread> pub_thread_{ nullptr };
};

TestPubComponent::TestPubComponent(const rclcpp::NodeOptions & options)
  : Node("TestPubComponent", options)
{
  auto input_file = this->declare_parameter("input", "/data/src.yuv");
  auto width = this->declare_parameter("width", 1920);
  auto height = this->declare_parameter("height", 1080);
  auto encoding = this->declare_parameter("encoding", "nv12");
  auto message_type = this->declare_parameter("test_type", "dmabuf_transport::type::Image");
  auto topic_name = this->declare_parameter("topic_name", "image");
  auto fps = this->declare_parameter("fps", 30);
  auto point_step = this->declare_parameter("point_step", 12);

  RCLCPP_INFO(get_logger(),
      "=== Publish type: %s, width: %ld, height: %ld, encoding: %s, input: %s, topic_name: %s, "
      "fps: %ld, point_step: %ld",
      message_type.c_str(), width, height, encoding.c_str(), input_file.c_str(), topic_name.c_str(),
      fps, point_step);

  if (message_type == "dmabuf_transport::type::Image") {
    image_pub_ = this->create_publisher<type::Image>(topic_name, 30);
    pub_thread_ = std::make_shared<std::thread>(
        &TestPubComponent::publish_image, this, width, height, encoding, fps, input_file);
  } else if (message_type == "sensor_msgs::msg::Image") {
    ros_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 30);
    pub_thread_ = std::make_shared<std::thread>(
        &TestPubComponent::publish_ros_image, this, width, height, encoding, fps, input_file);
  } else if (message_type == "dmabuf_transport::type::PointCloud2") {
    pcl_pub_ = this->create_publisher<type::PointCloud2>(topic_name, 30);
    pub_thread_ =
        std::make_shared<std::thread>(&TestPubComponent::publish_pcl, this, fps, input_file);
  } else if (message_type == "sensor_msgs::msg::PointCloud2") {
    ros_pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 30);
    pub_thread_ =
        std::make_shared<std::thread>(&TestPubComponent::publish_ros_pcl, this, fps, input_file);
  }
}

TestPubComponent::~TestPubComponent()
{
  if (pub_thread_->joinable()) {
    pub_thread_->join();
  }
}

void TestPubComponent::publish_image(int width,
    int height,
    const std::string & encoding,
    int fps,
    const std::string & data_file)
{
  auto data_size = 0;
  if (encoding == "nv12") {
    data_size = width * height * 1.5;
  } else if (encoding == "rgb8") {
    data_size = width * height * 3;
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "encoding: " << encoding << " not support");
    return;
  }

  auto data = read_data_from_file(data_file, data_size);
  if (data == nullptr) {
    return;
  }

  RCLCPP_ERROR(get_logger(), "start to publish");
  rclcpp::Rate rate(fps);
  while (rclcpp::ok()) {
    auto msg = std::make_unique<type::Image>();
    msg->width = width;
    msg->height = height;
    msg->encoding = encoding;

    auto dmabuf = lib_mem_dmabuf::DmaBuffer::alloc(data_size, "/dev/dma_heap/system");
    if (dmabuf == nullptr) {
      RCLCPP_ERROR_STREAM(get_logger(), "dmabuf alloc failed, size: " << data_size);
      return;
    }

    if (!dmabuf->set_data(data.get(), data_size)) {
      RCLCPP_ERROR(get_logger(), "dmabuf set data failed");
      return;
    }
    dmabuf->unmap();

    msg->dmabuf = dmabuf;
    msg->header.stamp = this->now();

    image_pub_->publish(std::move(msg));

    rate.sleep();
  }
}

void TestPubComponent::publish_ros_image(int width,
    int height,
    const std::string & encoding,
    int fps,
    const std::string & data_file)
{
  auto data_size = 0;
  auto src_step = 0;
  if (encoding == "nv12") {
    data_size = width * height * 1.5;
    src_step = width;
  } else if (encoding == "rgb8") {
    data_size = width * height * 3;
    src_step = width * 3;
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "encoding: " << encoding << " not support");
    return;
  }

  auto data = read_data_from_file(data_file, data_size);
  if (data == nullptr) {
    return;
  }

  rclcpp::Rate rate(fps);
  while (rclcpp::ok()) {
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->width = width;
    msg->height = height;
    msg->encoding = encoding;
    msg->step = src_step;

    msg->data.resize(data_size);
    std::memcpy(msg->data.data(), data.get(), data_size);

    msg->header.stamp = this->now();
    ros_image_pub_->publish(std::move(msg));

    rate.sleep();
  }
}

std::unique_ptr<char[]> TestPubComponent::read_data_from_file(const std::string & path,
    std::size_t size)
{
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) {
    RCLCPP_ERROR_STREAM(get_logger(), "open file: " << path << "failed");
    return nullptr;
  }

  auto data = std::make_unique<char[]>(size);
  auto read_size = file.readsome(data.get(), size);

  if (read_size != (long)size) {
    RCLCPP_ERROR_STREAM(get_logger(), "read file size: " << read_size << " != " << size);
    file.close();
    return nullptr;
  }

  file.close();
  return data;
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>> TestPubComponent::read_pcl_from_file(
    const std::string & path)
{
  auto p = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
  if (pcl::io::loadPLYFile(path, *p) == -1) {
    RCLCPP_ERROR_STREAM(get_logger(), "read ply file failed: " << path);
    return nullptr;
  }
  return p;
}

void TestPubComponent::publish_pcl(int fps, const std::string & data_file)
{
  auto p = read_pcl_from_file(data_file);
  if (p == nullptr) {
    return;
  }

  sensor_msgs::msg::PointCloud2 ros_msg;
  pcl::toROSMsg(*p, ros_msg);
  auto data_size = ros_msg.data.size();

  rclcpp::Rate rate(fps);
  int i = 1;
  while (rclcpp::ok()) {
    auto msg = std::make_unique<type::PointCloud2>();
    msg->width = ros_msg.width;
    msg->height = ros_msg.height;
    msg->point_step = ros_msg.point_step;
    msg->row_step = ros_msg.row_step;
    msg->fields = ros_msg.fields;
    msg->is_dense = ros_msg.is_dense;
    msg->is_bigendian = ros_msg.is_bigendian;

    auto dmabuf = lib_mem_dmabuf::DmaBuffer::alloc(data_size, "/dev/dma_heap/system");
    if (dmabuf == nullptr) {
      RCLCPP_ERROR_STREAM(get_logger(), "dmabuf alloc failed, size: " << data_size);
      return;
    }
    if (dmabuf->map()) {
      dmabuf->set_data(ros_msg.data.data(), data_size);
      dmabuf->unmap();
    } else {
      RCLCPP_ERROR(get_logger(), "dmabuf set data failed");
      return;
    }
    msg->dmabuf = dmabuf;

    msg->header.stamp = this->now();
    pcl_pub_->publish(std::move(msg));

    i++;
    if (i == fps) {
      i = 0;
    }
    rate.sleep();
  }
}

void TestPubComponent::publish_ros_pcl(int fps, const std::string & data_file)
{
  auto p = read_pcl_from_file(data_file);
  if (p == nullptr) {
    return;
  }

  rclcpp::Rate rate(fps);
  int i = 1;
  while (rclcpp::ok()) {
    auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*p, *msg);
    msg->header.stamp = this->now();
    ros_pcl_pub_->publish(std::move(msg));

    i++;
    if (i == fps) {
      i = 0;
    }
    rate.sleep();
  }
}

}  // namespace dmabuf_transport

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dmabuf_transport::TestPubComponent)
