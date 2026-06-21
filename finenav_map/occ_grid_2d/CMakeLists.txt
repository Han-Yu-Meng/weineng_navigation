cmake_minimum_required(VERSION 3.22)
project(finenav_map_occ_grid_2d)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# ── 从 YAML 生成参数结构体（occ_grid_2d::Params / occ_grid_2d::ParamListener）──
generate_parameter_library(
    occ_grid_2d_params
    param/occ_grid_2d.yaml
)

# ── 地图库（非 header-only，因为依赖 OpenCV）────────────────────────────
add_library(${PROJECT_NAME}
    src/occ_grid_2d_map.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/occ_grid_2d_params_parameters>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    occ_grid_2d_params
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
    finenav_core
    finenav_util
    nav_msgs
    rclcpp
)

# ── 安装 ─────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME} occ_grid_2d_params
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(finenav_core finenav_util nav_msgs rclcpp)

ament_auto_package()
