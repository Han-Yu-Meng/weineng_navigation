cmake_minimum_required(VERSION 3.22)
project(finenav_localizer)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(finenav_util REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)
find_package(generate_parameter_library REQUIRED)

include_directories(include)

# Generate parameter library for FineNavLocalizer
generate_parameter_library(
        localizer_params
        src/finenav_localizer.yaml
)

# FineNav Localizer (ESKF-based localization module)
add_library(${PROJECT_NAME} SHARED
        src/finenav_localizer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        tf2_ros
        tf2_eigen
        geometry_msgs
        finenav_util
)
target_link_libraries(${PROJECT_NAME}
        Eigen3::Eigen
        Sophus::Sophus
        localizer_params
)

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    add_subdirectory(test)
    add_subdirectory(eval)
endif ()

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
# Install the generated parameter header so downstream packages can find it
install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/include/finenav_localizer/localizer_params.hpp
        DESTINATION include/finenav_localizer
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
        rclcpp
        tf2_ros
        tf2_eigen
        geometry_msgs
        finenav_util
        Eigen3
        Sophus
        generate_parameter_library
)
ament_package()
