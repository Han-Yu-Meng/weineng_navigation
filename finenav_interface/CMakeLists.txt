cmake_minimum_required(VERSION 3.22)
project(finenav_interface)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)


include_directories(include)

ament_auto_find_build_dependencies()

generate_parameter_library(
        test_map_params
        src/test_map.yaml
)
generate_parameter_library(
        test_planner_params
        src/test_planner.yaml
)

#ament_auto_add_executable(simple_planner_node src/local_layer_node.cpp)
#target_link_libraries(simple_planner_node test_map_params test_planner_params)

#ament_auto_add_library(finenav_manager src/finenav_manager.cpp)
ament_auto_add_executable(test_interface_node src/test.cpp)
ament_auto_add_executable(grid_map_stvl_node src/grid_map_stvl.cpp)
target_link_libraries(test_interface_node test_map_params test_planner_params)
target_link_libraries(grid_map_stvl_node test_map_params test_planner_params tf2::tf2 tf2_ros::tf2_ros tf2_eigen::tf2_eigen)


ament_auto_package()