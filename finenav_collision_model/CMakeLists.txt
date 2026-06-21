cmake_minimum_required(VERSION 3.22)
project(finenav_collision_model)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

add_library(${PROJECT_NAME} SHARED
    src/obb_collision_model.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen
)

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_find_gtest()
    ament_add_gtest(${PROJECT_NAME}_test
        test/obb_collision_model_test.cpp
    )
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif ()

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    eigen3_cmake_module
    Eigen3
)

ament_auto_package()
