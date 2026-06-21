cmake_minimum_required(VERSION 3.22)
project(finenav_engine) # Header-only

find_package(ament_cmake REQUIRED)

set(finenav_dependencies
        finenav_core
        finenav_localizer
        finenav_navigator
        finenav_motion_decorator
        finenav_msgs
        finenav_util
)

foreach(dep IN LISTS finenav_dependencies)
    find_package(${dep} REQUIRED)
endforeach()

install(DIRECTORY include/
        DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(${finenav_dependencies})

ament_package()