cmake_minimum_required(VERSION 3.16)
project(libserial_vendor)

find_package(ament_cmake REQUIRED)

add_subdirectory(libserial-1.0)


ament_export_include_directories(libserial-1.0/src/libserial)
ament_export_libraries(libserial-1.0)
ament_package()
