# cmake needs this line
cmake_minimum_required(VERSION 3.1)

# Define project name
project(Robotronikaruco)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)

#define location of nvmmapi and argus in jetson
set(MMAPI_DIR "/usr/src/jetson_multimedia_api")

set(ARGUS_DIR "${MMAPI_DIR}/argus")
set(ARGUS_INCLUDE_DIR "${ARGUS_DIR}/")

#add rules to find nvmmapi and stuff
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" "${CMAKE_MODULE_PATH}")

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

find_package(NVMMAPI)
if(NVMMAPI_FOUND)
    find_package(Argus REQUIRED)
    find_package(OpenGLES REQUIRED)
    find_package(EGL REQUIRED)
    include_directories(
        ${ARGUS_INCLUDE_DIR}
        ${EGL_INCLUDE_DIR}
        ${OPENGLES_INCLUDE_DIR}
        ${ARGUS_DIR}/samples/utils)
endif(NVMMAPI_FOUND)

include_directories(
    $OPENCV_INCLUDE_DIRS
    ./include
    )

file(GLOB_RECURSE SOURCES "source/*.cpp")
add_executable(Robotronikaruco ${SOURCES})

message(STATUS "Sources found : ${SOURCES}")

set_target_properties(Robotronikaruco PROPERTIES CXX_STANDARD 17)

# Link your application with OpenCV libraries
target_link_libraries(Robotronikaruco PRIVATE ${OpenCV_LIBS} stdc++fs)