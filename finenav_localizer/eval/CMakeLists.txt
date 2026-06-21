add_executable(${PROJECT_NAME}_eval
        eval_scenario.cpp
)
target_include_directories(${PROJECT_NAME}_eval PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
)
target_link_libraries(${PROJECT_NAME}_eval
        finenav_localizer
)
