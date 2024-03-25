cmake_minimum_required(VERSION 3.8)
project(dmabuf_transport)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/type/image.cpp
  src/type/point_cloud2.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_auto_add_gtest(${PROJECT_NAME}_image_test_pub
    test/image_test.cpp
  )
  target_compile_definitions(${PROJECT_NAME}_image_test_pub PRIVATE "-DTEST_PUBLISHER")

  ament_auto_add_gtest(${PROJECT_NAME}_image_test_sub
    test/image_test.cpp
  )
  target_compile_definitions(${PROJECT_NAME}_image_test_sub PRIVATE "-DTEST_SUBSCRIBER")

  ament_auto_add_gtest(${PROJECT_NAME}_point_cloud2_test_pub
    test/point_cloud2_test.cpp
  )
  target_compile_definitions(${PROJECT_NAME}_point_cloud2_test_pub PRIVATE "-DTEST_PUBLISHER")

  ament_auto_add_gtest(${PROJECT_NAME}_point_cloud2_test_sub
    test/point_cloud2_test.cpp
  )
  target_compile_definitions(${PROJECT_NAME}_point_cloud2_test_sub PRIVATE "-DTEST_SUBSCRIBER")

  # for end-to-end test
  find_package(rclcpp_components REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(PCL REQUIRED)
  macro(add_test_component component_name class_name)
    ament_auto_add_library(${PROJECT_NAME}_${component_name}_component SHARED
      test/${component_name}.cpp
    )
    ament_target_dependencies(${PROJECT_NAME}_${component_name}_component
      rclcpp_components
      pcl_conversions
    )
    target_include_directories(${PROJECT_NAME}_${component_name}_component PUBLIC
      ${PCL_INCLUDE_DIRS}
    )
    target_link_libraries(${PROJECT_NAME}_${component_name}_component
      ${PCL_LIBRARIES}
    )
    rclcpp_components_register_nodes(${PROJECT_NAME}_${component_name}_component ${class_name})
  endmacro(add_test_component)

  add_test_component(test_pub "dmabuf_transport::TestPubComponent")
  add_test_component(test_sub "dmabuf_transport::TestSubComponent")
  add_test_component(test_dump "dmabuf_transport::TestDumpComponent")

endif()

ament_auto_package()
