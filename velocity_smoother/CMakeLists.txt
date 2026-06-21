cmake_minimum_required(VERSION 3.22)
project(velocity_smoother)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(velocity_smoother_node src/velocity_smoother.cpp)
ament_target_dependencies(velocity_smoother_node
    rclcpp
    geometry_msgs
)

target_include_directories(velocity_smoother_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS velocity_smoother_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
