cmake_minimum_required(VERSION 3.22)
project(finenav_motion_decorator)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    src/trim_by_count.cpp
    src/trim_by_duration.cpp
    src/trim_by_distance.cpp
    src/seek_to_nearest_point.cpp
    src/trim_by_aabb.cpp
    src/smooth_path.cpp
)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

#if (BUILD_TESTING)
#    find_package(ament_cmake_gtest REQUIRED)
#    ament_find_gtest()
#    add_subdirectory(test)
#endif ()

ament_auto_package()