# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(PROJECT_VER "2.0.5")

# Add this line to disable the specific warning
add_compile_options(-Wno-missing-field-initializers)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(xiaozhi)

# Custom target to create merged binary for easy flashing
add_custom_target(merged-bin
    COMMAND ${CMAKE_COMMAND} -E echo "Creating merged binary file..."
    COMMAND python $ENV{IDF_PATH}/components/esptool_py/esptool/esptool.py 
        --chip ${IDF_TARGET}
        merge-bin
        -o ${CMAKE_PROJECT_NAME}-merged.bin
        --flash-mode dio
        --flash-freq 80m
        --flash-size 16MB
        @flash_args
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    DEPENDS app
    COMMENT "Merging binary files into ${CMAKE_PROJECT_NAME}-merged.bin"
    VERBATIM
)

