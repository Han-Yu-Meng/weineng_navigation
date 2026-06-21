cmake_minimum_required(VERSION 3.22)
project(finenav_core) # Header-only

find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(finenav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(finenav_util REQUIRED)
find_package(generate_parameter_library REQUIRED)

set(ros_dependencies
        eigen3_cmake_module
        rclcpp
        rclcpp_action
        tf2_ros
        tf2_eigen
        finenav_msgs
        sensor_msgs
        geometry_msgs
        finenav_util
)

# Generate parameter library for NavStateManager — moved to finenav_navigator

install(DIRECTORY include/
        DESTINATION include/
)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)
target_link_libraries(${PROJECT_NAME} INTERFACE
        fmt::fmt
        Eigen3::Eigen
)
ament_target_dependencies(${PROJECT_NAME} INTERFACE
        ${ros_dependencies}
)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        INCLUDES DESTINATION include
)
ament_export_dependencies(
        fmt
        Eigen3
        generate_parameter_library
        ${ros_dependencies}
)

ament_package()