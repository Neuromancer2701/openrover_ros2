add_executable(vesc_util main.cpp)

target_include_directories(vesc_util
        PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_link_libraries(vesc_util
        PUBLIC
        vesc
)

install(
        TARGETS
        vesc_util
        EXPORT ${PROJECT_NAME}-targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)