ament_add_gtest(${PROJECT_NAME}_test
        test_localizer_units.cpp
)
target_link_libraries(${PROJECT_NAME}_test
        finenav_localizer
)
