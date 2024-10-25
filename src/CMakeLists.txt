set(CAMERA_SERVER_EXE_NAME camera_server_node)
set(CAMERA_SERVER_EXE_SOURCES ${CAMERA_SERVER_EXE_NAME}.cpp)

add_executable(${CAMERA_SERVER_EXE_NAME} ${CAMERA_SERVER_EXE_SOURCES})
ament_target_dependencies(${CAMERA_SERVER_EXE_NAME} rclcpp)
target_link_libraries(${CAMERA_SERVER_EXE_NAME} camera_driver)

install(TARGETS
    ${CAMERA_SERVER_EXE_NAME}
    DESTINATION lib/${PROJECT_NAME})