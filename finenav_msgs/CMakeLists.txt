cmake_minimum_required(VERSION 3.22)
project(finenav_msgs)

find_package(ament_cmake_auto REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)

ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
        msg/Trajectory.msg
        msg/RobotState.msg
        msg/NavStatus.msg
        action/ComputePlan.action
        action/NavigateToPose.action
        DEPENDENCIES geometry_msgs std_msgs
)

ament_auto_package()

