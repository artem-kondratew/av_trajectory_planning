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

add_executable(acc_node
    src/acc_mpc.cpp
    src/acc_node.cpp
)

add_executable(pp_node
    src/pp_node.cpp
)

add_executable(stanley_node
    src/stanley_node.cpp
)

add_executable(lateral_mpc_node
    src/lateral_mpc.cpp
    src/lateral_mpc_node.cpp
)

add_executable(steer_test_node
    src/steer_test_node.cpp
)

target_include_directories(cc_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(acc_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(pp_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(stanley_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(lateral_mpc_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(cc_node
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
)

target_link_libraries(acc_node
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
)

target_link_libraries(lateral_mpc_node
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
)

ament_target_dependencies(cc_node carla_msgs control_toolbox geometry_msgs rclcpp sensor_msgs std_msgs)
ament_target_dependencies(acc_node carla_msgs control_toolbox geometry_msgs rclcpp sensor_msgs std_msgs)
ament_target_dependencies(pp_node carla_msgs rclcpp)
ament_target_dependencies(stanley_node carla_msgs rclcpp)
ament_target_dependencies(lateral_mpc_node carla_msgs control_toolbox geometry_msgs rclcpp sensor_msgs std_msgs)
ament_target_dependencies(steer_test_node carla_msgs geometry_msgs rclcpp)

install(DIRECTORY
    launch config
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    cc_node
    acc_node
    pp_node
    stanley_node
    lateral_mpc_node
    steer_test_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
