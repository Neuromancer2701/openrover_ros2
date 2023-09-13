cmake_minimum_required(VERSION 3.16)
project(libserial_vendor)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_subdirectory(libserial-1.0)

#ament_export_include_directories(${DOCOPT_INCLUDE_DIRS})
ament_export_libraries(libserial)

ament_package()
