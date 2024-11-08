set(CAMERA_SERVER_EXE_SOURCES ${CAMERA_SERVER_EXE_NAME}.cpp)
set(CAMERA_CLIENT_EXE_SOURCES ${CAMERA_CLIENT_EXE_NAME}.cpp)

add_executable(${CAMERA_SERVER_EXE_NAME} ${CAMERA_SERVER_EXE_SOURCES})
add_executable(${CAMERA_CLIENT_EXE_NAME} ${CAMERA_CLIENT_EXE_SOURCES})

ament_target_dependencies(${CAMERA_SERVER_EXE_NAME}
    cv_bridge
    rclcpp
    sensor_msgs
    std_msgs)
target_link_libraries(${CAMERA_SERVER_EXE_NAME}
        camera_driver
        ${OpenCV_LIBS})

ament_target_dependencies(${CAMERA_CLIENT_EXE_NAME}
    rclcpp
    sensor_msgs)
target_link_libraries(${CAMERA_CLIENT_EXE_NAME}
    camera_driver
    ${OpenCV_LIBS})

install(TARGETS
    ${CAMERA_SERVER_EXE_NAME}
    ${CAMERA_CLIENT_EXE_NAME}
    DESTINATION lib/${PROJECT_NAME})