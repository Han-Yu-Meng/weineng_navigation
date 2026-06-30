cmake_minimum_required(VERSION 3.22)
project(finenav_demo)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

set(dependencies
        ament_index_cpp
        rclcpp
        rclcpp_action
        finenav_engine
        finenav_map_grid_map
        finenav_map_occ_grid_2d
        finenav_mppi_controller
        finenav_route_planner
        finenav_rotate_shim
        finenav_core
        finenav_collision_model
        finenav_motion_decorator
        nav_msgs
        geometry_msgs
        sensor_msgs
        std_srvs
        tf2
        tf2_eigen
        tf2_geometry_msgs
        message_filters
        pcl_conversions
)

foreach(dep IN LISTS dependencies)
    find_package(${dep} REQUIRED)
endforeach()

add_subdirectory(stvl_with_mppi)

ament_package()