// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef DMABUF_TRANSPORT__DMABUF_TYPE_ADAPTER
#define DMABUF_TRANSPORT__DMABUF_TYPE_ADAPTER

#include "dmabuf_transport/dmabuf.hpp"

namespace dmabuf_transport
{

template <typename T>
struct DmaBufTypeAdapter
{
  explicit DmaBufTypeAdapter() = default;
  ~DmaBufTypeAdapter() {}

  const std::shared_ptr<DmaBuffer> get_dma_buf() const { return buffer_; }
  void set_dma_buf(std::shared_ptr<DmaBuffer> buf) { buffer_ = buf; }

  const T get_ros_message() const { return ros_message_; }
  void set_ros_message(const T& message) { ros_message_ = message; }

private:
  T ros_message_;
  std::shared_ptr<DmaBuffer> buffer_{ nullptr };
};

}  // namespace dmabuf_transport

#endif  // DMABUF_TRANSPORT__DMABUF_TYPE_ADAPTER