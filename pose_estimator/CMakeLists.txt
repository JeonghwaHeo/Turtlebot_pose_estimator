cmake_minimum_required(VERSION 3.5)
project(pose_estimator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenMP REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(pose_estimator src/pose_estimator.cpp)

if(OpenMP_CXX_FOUND)
  add_compile_options(-fopenmp)
  target_link_libraries(pose_estimator OpenMP::OpenMP_CXX)
endif()

ament_target_dependencies(pose_estimator rclcpp sensor_msgs)

install(TARGETS
  pose_estimator
  DESTINATION lib/${PROJECT_NAME})
ament_package()
