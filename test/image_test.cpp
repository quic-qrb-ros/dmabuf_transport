// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "dmabuf_transport/type/image.hpp"

class PubNode : public rclcpp::Node
{
public:
  explicit PubNode() : Node("image_test_pub")
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "image_test_pub start");

    // create publisher
    rclcpp::QoS qos{ 10 };
    qos.transient_local();
    pub_ = this->create_publisher<dmabuf_transport::type::Image>("image", qos);

    int count = 10;
    while (count-- > 0) {
      // create message
      auto msg = std::make_unique<dmabuf_transport::type::Image>();
      msg->header = std_msgs::msg::Header();

      auto dmabuf = lib_mem_dmabuf::DmaBuffer::alloc(1024, "/dev/dma_heap/system");
      if (dmabuf == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("image_test"), "dma buffer alloc failed");
      }

      char mock_data[1024]{ "this is test data" };

      if (!dmabuf->set_data(mock_data, 1024)) {
        RCLCPP_ERROR(rclcpp::get_logger("image_test"), "save data to dma buffer failed");
        return;
      }
      dmabuf->unmap();
      msg->dmabuf = dmabuf;

      // publish image
      RCLCPP_INFO_STREAM(this->get_logger(), "publish image data, fd: " << dmabuf->fd());
      pub_->publish(std::move(msg));

      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }

private:
  rclcpp::Publisher<dmabuf_transport::type::Image>::SharedPtr pub_;
};

class SubNode : public rclcpp::Node
{
public:
  explicit SubNode() : Node("image_test_sub")
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "image_test_sub start");

    rclcpp::QoS qos{ 10 };
    qos.transient_local();

    sub_ = this->create_subscription<dmabuf_transport::type::Image>(
        "image", qos, [this](const std::shared_ptr<dmabuf_transport::type::Image> msg) {
          RCLCPP_INFO_STREAM(this->get_logger(), "got message: image fd: " << msg->dmabuf->fd());
        });
  }

private:
  rclcpp::Subscription<dmabuf_transport::type::Image>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

#ifdef TEST_PUBLISHER
  rclcpp::spin(std::make_shared<PubNode>());
#endif

#ifdef TEST_SUBSCRIBER
  rclcpp::spin(std::make_shared<SubNode>());
#endif

  rclcpp::shutdown();

  return 0;
}
