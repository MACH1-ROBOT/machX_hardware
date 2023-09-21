# Build and install camera_driver package
add_subdirectory(camera_driver)

# Add a custom target to install the gpiozero_plus Python module
add_custom_target(
    install_gpiozero_plus ALL
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/gpiozero_plus/setup.py sdist
    COMMAND pip install --no-warn-script-location .
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/gpiozero_plus
    COMMENT "Installing gpiozero_plus library."
)