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

# BT Plugin: PublishNavigationState
add_library(finenav_publish_navigation_state SHARED
    src/bt_plugins/publish_navigation_state.cpp
)
ament_target_dependencies(finenav_publish_navigation_state ${dependencies})
target_include_directories(finenav_publish_navigation_state PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_definitions(finenav_publish_navigation_state PRIVATE BT_PLUGIN_EXPORT)

# BT Plugin: NavigationControl
add_library(finenav_navigation_control SHARED
    src/bt_plugins/navigation_control.cpp
)
ament_target_dependencies(finenav_navigation_control ${dependencies})
target_include_directories(finenav_navigation_control PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_definitions(finenav_navigation_control PRIVATE BT_PLUGIN_EXPORT)

add_executable(hello_finenav src/apps/hello_finenav.cpp)
target_link_libraries(hello_finenav terrain_lib)
ament_target_dependencies(hello_finenav ${dependencies})

add_executable(goal_pose_bridge_node src/apps/goal_pose_bridge_node.cpp)
ament_target_dependencies(goal_pose_bridge_node ${dependencies})
target_link_libraries(goal_pose_bridge_node ${std_srvs_LIBRARIES})

install(TARGETS terrain_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS finenav_publish_navigation_state finenav_navigation_control
        ARCHIVE DESTINATION lib/plugins
        LIBRARY DESTINATION lib/plugins
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

install(DIRECTORY behavior_tree
        DESTINATION share/${PROJECT_NAME}
)
