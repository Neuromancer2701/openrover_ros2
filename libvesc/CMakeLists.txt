cmake_minimum_required(VERSION 3.8)
project(libvesc)

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

ament_package()
