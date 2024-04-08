cmake_minimum_required(VERSION 3.8)
project(vesc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


# libserial:
# Following linking instructions from: https://github.com/crayzeewulf/libserial/blob/master/examples/example_project/CMakeLists.txt
set(THREADS_HAVE_PTHREAD_ARG 1)
find_package(Threads REQUIRED)

# Include libserial for serial communication
# Typical find_package() does not work for libserial, for reference see:
# https://github.com/crayzeewulf/libserial/issues/113#issuecomment-432245159
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

include_directories(include
        ${SERIAL_INCLUDE_DIRS})

file(GLOB VESC_SOURCES CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${VESC_SOURCES} )

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>

        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/libvesc
        )

target_compile_options(${PROJECT_NAME}
        PRIVATE -Wall -Wextra -Wpedantic -Werror)


##############
## Install  ##
##############

install(
        TARGETS ${PROJECT_NAME}
        EXPORT  export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(
        DIRECTORY include/
        DESTINATION include
        PATTERN "*~" EXCLUDE
        PATTERN "*.in" EXCLUDE)


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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
        ament_cmake
)

ament_package()
