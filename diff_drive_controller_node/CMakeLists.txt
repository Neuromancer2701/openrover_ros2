cmake_minimum_required(VERSION 3.8)
project(diff_drive_controller_node)

# Default to C++17
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# For clang-tidy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(vesc REQUIRED)

include_directories(
        include
)

add_library(diff_drive_controller SHARED
        src/diff_drive_controller_node.cpp
)

target_compile_options(diff_drive_controller
        PRIVATE
        -Wall -Wextra -Wpedantic -Werror
)

set(DEPENDENCIES
        rclcpp
        std_msgs
        geometry_msgs
        rclcpp_lifecycle
        lifecycle_msgs
        vesc
)

ament_target_dependencies(diff_drive_controller ${DEPENDENCIES})

add_executable(diff_drive_controller_node
        src/diff_drive_controller_main.cpp)

target_link_libraries(diff_drive_controller_node diff_drive_controller)

target_compile_options(diff_drive_controller_node
        PRIVATE
        -Wall -Wextra -Wpedantic -Werror
)

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
        diff_drive_controller

        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(TARGETS diff_drive_controller_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories( include )
ament_export_dependencies( ${DEPENDENCIES} )
ament_export_libraries( diff_drive_controller )
ament_export_targets( export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
