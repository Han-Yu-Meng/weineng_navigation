cmake_minimum_required(VERSION 3.22)
project(finenav_util)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)

include_directories(include)

ament_auto_find_build_dependencies()

#add_library(${PROJECT_NAME} INTERFACE)
#
#target_include_directories(${PROJECT_NAME} INTERFACE
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#        $<INSTALL_INTERFACE:include>
#)

ament_auto_add_library(${PROJECT_NAME} SHARED
        src/node_thread.cpp
        src/cloud_publish_helper.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)


#if (BUILD_TESTING)
#    find_package(ament_cmake_gtest REQUIRED)
#    ament_find_gtest()
#    add_subdirectory(test)
#endif ()

ament_auto_package() # TODO: 之后讨论下是否添加 USE_SCOPED_HEADER_INSTALL_DIR 的问题