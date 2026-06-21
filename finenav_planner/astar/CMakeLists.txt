cmake_minimum_required(VERSION 3.22)
project(finenav_planner_astar)
set(CMAKE_CXX_STANDARD 20)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(include ${OpenCV_INCLUDE_DIRS})
ament_auto_find_build_dependencies()
# ── parameter library (generated from astar.yaml) ──────────────────────────
generate_parameter_library(
    astar_params
    astar.yaml
)
# ── planner library ─────────────────────────────────────────────────────────
ament_auto_add_library(astar_planner_lib
        src/astar_algorithm.cpp
        src/astar_path_search.cpp
        src/occ_grid_map_view.cpp
)
target_link_libraries(astar_planner_lib astar_params ${OpenCV_LIBS})
# ── eval executable ─────────────────────────────────────────────────────────
ament_auto_add_executable(astar_eval
    eval/astar_eval.cpp
        src/astar_algorithm.cpp
        src/astar_path_search.cpp
)
target_include_directories(astar_eval PRIVATE eval test)
target_link_libraries(astar_eval astar_params)
# ── unit tests ──────────────────────────────────────────────────────────────
if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(astar_test
        test/astar_test.cpp
            src/astar_algorithm.cpp
            src/astar_path_search.cpp
    )
    # Inherit the same includes and linked libs as the planner library
    ament_target_dependencies(astar_test
        finenav_core
        finenav_util
        finenav_msgs
        rclcpp
    )
    target_include_directories(astar_test PRIVATE test)
    target_link_libraries(astar_test astar_params)
endif ()
ament_auto_package()
