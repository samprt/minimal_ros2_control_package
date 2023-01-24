cmake_minimum_required(VERSION 3.8)
project(minimal_example_ros2_control)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(generate_parameter_library REQUIRED)

# Hardware component
add_library(minimal_example_system SHARED
        src/minimal_example_system.cpp)
target_include_directories(minimal_example_system PRIVATE
        include)
ament_target_dependencies(minimal_example_system
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        )
pluginlib_export_plugin_description_file(hardware_interface minimal_example_system.xml)

# Controller
generate_parameter_library(
        minimal_example_controller_parameters
        src/minimal_example_controller_parameters.yaml
)
add_library(minimal_example_controller SHARED
        src/minimal_example_controller.cpp)
target_include_directories(minimal_example_controller PRIVATE
        include)
target_link_libraries(minimal_example_controller
        minimal_example_controller_parameters)
ament_target_dependencies(minimal_example_controller
        controller_interface
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        )
pluginlib_export_plugin_description_file(controller_interface minimal_example_controller.xml)


install(
        DIRECTORY include/
        DESTINATION include
)

install(TARGETS
        minimal_example_system
        minimal_example_controller
        minimal_example_controller_parameters
        DESTINATION lib
        )

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})

install(
        TARGETS
        minimal_example_system
        minimal_example_controller
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

if (BUILD_TESTING)

endif ()

ament_export_include_directories(
        include
)
ament_export_libraries(
        minimal_example_system
        minimal_example_controller
)
ament_export_dependencies(
        hardware_interface
        controller_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
)
ament_package()

