cmake_minimum_required(VERSION 3.22)
project(finenav_navigator)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(finenav_msgs REQUIRED)
find_package(finenav_util REQUIRED)
find_package(finenav_localizer REQUIRED)

include_directories(include)

set(dependencies
        rclcpp
        rclcpp_action
        ament_index_cpp
        behaviortree_cpp
        generate_parameter_library
        eigen3_cmake_module
        geometry_msgs
        finenav_msgs
        finenav_util
        finenav_localizer
)

# Library
generate_parameter_library(
        navigator_params
        src/finenav_navigator.yaml
)

add_library(finenav_navigator STATIC
        src/naviagtor.cpp
        src/behavior_tree_engine.cpp
)
ament_target_dependencies(finenav_navigator
        ${dependencies}
)
target_link_libraries(finenav_navigator
        navigator_params
        Eigen3::Eigen
)

# Action Plugins
add_library(finenav_compute_plan_behavior SHARED
        src/plugins/action/compute_plan.cpp
)
ament_target_dependencies(finenav_compute_plan_behavior
        ${dependencies}
)
target_compile_definitions(finenav_compute_plan_behavior PRIVATE BT_PLUGIN_EXPORT)

# Condition Plugins (pure BT nodes, no ROS params needed)
add_library(finenav_navigator_conditions SHARED
        src/plugins/condition/conditions.cpp
)
ament_target_dependencies(finenav_navigator_conditions
        ${dependencies}
)
target_compile_definitions(finenav_navigator_conditions PRIVATE BT_PLUGIN_EXPORT)

# PublishZeroCmdVel Plugin
add_library(finenav_publish_zero_cmd_vel_behavior SHARED
        src/plugins/action/publish_zero_cmd_vel.cpp
)
ament_target_dependencies(finenav_publish_zero_cmd_vel_behavior
        ${dependencies}
)
target_compile_definitions(finenav_publish_zero_cmd_vel_behavior PRIVATE BT_PLUGIN_EXPORT)

# Install
install(TARGETS finenav_navigator
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS finenav_compute_plan_behavior finenav_navigator_conditions finenav_publish_zero_cmd_vel_behavior
        ARCHIVE DESTINATION lib/plugins
        LIBRARY DESTINATION lib/plugins
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

# Export
ament_export_include_directories(include)
ament_export_libraries(finenav_navigator)
ament_export_dependencies(${dependencies} Eigen3)

ament_package()