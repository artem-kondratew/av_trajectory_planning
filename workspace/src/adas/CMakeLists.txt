cmake_minimum_required(VERSION 3.8)
project(adas)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -Ofast")

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(cc_node
    src/cc_mpc.cpp
    src/cc_node.cpp
)

target_include_directories(cc_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(cc_node
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
)

ament_target_dependencies(cc_node carla_msgs control_toolbox geometry_msgs rclcpp sensor_msgs std_msgs)

install(DIRECTORY
    launch config
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    cc_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
