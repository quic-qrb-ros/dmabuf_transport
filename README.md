# Dmabuf Transport

dmabuf_transport is a package for zero-copy transport ROS message with Linux dma-buf file descriptor.

## Overview

[Dmabuf Transport](https://github.com/quic-qrb-ros/dmabuf_transport) provides a way to share data between different hardware accelerators and different ROS nodes with zero-copy. 

It is built on ROS 2 [Type Adaption](https://ros.org/reps/rep-2007.html). It allows us to define methods for serializing directly to the user requested type, and/or using that type in intra-process communication without ever converting it.

We can define our adapted types with Linux dma-buf, and send dma-buf descriptor with ROS. Then use [ROS Composition](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Composition.html) to confirm ROS nodes running in a single process, it enables zero-copy over the pipeline.

## System Requirements

- Linux kernel version 5.12 and later, for kernel dma-buf support.
- ROS 2 Humble and later, for type adaption support.

## QuickStart

## Code Sync and Build

Currently, we only support build with QCLINUX SDK.

1. Setup QCLINUX SDK environments follow this document: [Set up the cross-compile environment](https://docs.qualcomm.com/bundle/publicresource/topics/80-65220-2/develop-your-first-application_6.html?product=1601111740013072&facet=Qualcomm%20Intelligent%20Robotics%20(QIRP)%20Product%20SDK&state=releasecandidate)

2. Create `ros_ws` directory in `<qirp_decompressed_workspace>/qirp-sdk/`

     ```bash
     mkdir -p <qirp_decompressed_workspace>/qirp-sdk/ros_ws
     ```

3. Clone this repository and dependencies under `<qirp_decompressed_workspace>/qirp-sdk/ros_ws`

     ```bash
     cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws
     git clone https://github.com/quic-qrb-ros/lib_mem_dmabuf.git
     git clone https://github.com/quic-qrb-ros/dmabuf_transport.git
     ```

4. Build projects

     ```bash
     export AMENT_PREFIX_PATH="${OECORE_TARGET_SYSROOT}/usr;${OECORE_NATIVE_SYSROOT}/usr"
     export PYTHONPATH=${PYTHONPATH}:${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages
     
     colcon build --merge-install --cmake-args \
       -DPython3_ROOT_DIR=${OECORE_TARGET_SYSROOT}/usr \
       -DPython3_NumPy_INCLUDE_DIR=${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages/numpy/core/include \
       -DPYTHON_SOABI=cpython-310-aarch64-linux-gnu -DCMAKE_STAGING_PREFIX=$(pwd)/install \
       -DCMAKE_PREFIX_PATH=$(pwd)/install/share \
       -DBUILD_TESTING=OFF
     ```
## Using dmabuf_transport in your project

1. Add dependency in package.xml

   ```xml
   <depend>dmabuf_transport</depend>
   ```

2. Add dependency in CMakeLists.txt

   ```cmake
   find_package(dmabuf_transport REQUIRED)

   ament_target_dependencies(${PROJECT_NAME}
     # ...
     dmabuf_transport
   )
   ```

3. Zero-copy transport dmabuf_transport types

   ```c++
   #include "dmabuf_transport/type/image.hpp"
   // create message
   auto msg = std::make_unique<dmabuf_transport::type::Image>();
   msg->header = std_msgs::msg::Header();
   // save message data to dmabuf
   auto dmabuf = lib_mem_dmabuf::DmaBuffer::alloc(1024, "/dev/dma_heap/system");
   // ... set data
   msg->dmabuf = dmabuf;

   // publish message
   pub_->publish(std::move(msg));
   ```

- Check [test](./test/) directory to find more details.

## Supported Types

The following table lists current supported types:

| Zero Copy Transport Type                                     | ROS Interface                                                |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| [dmabuf_transport::type::Image](./include/dmabuf_transport/type/image.hpp) | [sensor_msgs::msg::Image](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Image.msg) |
| [dmabuf_transport::type::PointCloud2](./include/dmabuf_transport/type/pointcloud2.hpp) | [sensor_msgs::msg::Imu](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg) |

## Supported Platforms

This package is designed and tested to be compatible with ROS 2 Humble running on Qualcom RB3 gen2.

| Hardware                                                     | Software          |
| ------------------------------------------------------------ | ----------------- |
| [Qualcomm RB3 gen2](https://www.qualcomm.com/developer/hardware/rb3-gen-2-development-kit) | LE.QCROBOTICS.1.0 |

## Resources

- [ROS 2 Type Adaption](https://ros.org/reps/rep-2007.html): ROS 2 new feature to implement zero copy transport.
- [Linux dma-buf](https://docs.kernel.org/driver-api/dma-buf.html): Linux kernel subsystem for sharing buffers for hardware (DMA) access across multiple device drivers and subsystems, and for synchronizing asynchronous hardware access
- [lib_mem_dmabuf](https://github.com/quic-qrb-ros/lib_mem_dmabuf): Library for access and interact with Linux DMA heaps.

## Contributions

Thanks for your interest in contributing to dmabuf_transport! Please read our [Contributions Page](CONTRIBUTING.md) for more information on contributing features or bug fixes. We look forward to your participation!

## License

dmabuf_transport is licensed under the BSD 3-clause "New" or "Revised" License.

Check out the [LICENSE](LICENSE) for more details.
