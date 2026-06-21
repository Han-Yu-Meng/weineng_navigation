add_library(terrain_lib
    src/terrain/terrain_analysis.cpp
    src/terrain/terrain_mapview.cpp
    src/stvl/stvl_manager.cpp
    src/occ_grid/occ_grid_map_view.cpp
)
ament_target_dependencies(terrain_lib ${dependencies})

# terrain_mapview.hpp uses OpenCV and PCL
target_link_libraries(terrain_lib
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
)

target_include_directories(terrain_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)


add_executable(hello_finenav src/apps/hello_finenav.cpp)
target_link_libraries(hello_finenav terrain_lib)
ament_target_dependencies(hello_finenav ${dependencies})

add_executable(goal_pose_bridge_node src/apps/goal_pose_bridge_node.cpp)
ament_target_dependencies(goal_pose_bridge_node ${dependencies})

install(TARGETS terrain_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS hello_finenav
        DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS goal_pose_bridge_node
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)
