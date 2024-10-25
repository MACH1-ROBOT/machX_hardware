message(STATUS "*****************************LIB - ${CAMERA_SERVER_EXE_NAME}")
message(STATUS "*****************************PWD- ${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(${CAMERA_SERVER_EXE_NAME} PRIVATE
	"${CMAKE_CURRENT_SOURCE_DIR}"
	${OpenCV_INCLUDE_DIRS}
)