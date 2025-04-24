cmake_minimum_required(VERSION 3.8)
project(otoshidama_swerve_controller)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

target_include_directories(${PROJECT_NAME}
    SYSTEM PUBLIC
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "otoshidama_swerve_controller::twist_string_to_canable"
    EXECUTABLE twist_string_to_canable
)

ament_auto_package(
    INSTALL_TO_SHARE
)