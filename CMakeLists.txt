# cmake needs this line
cmake_minimum_required(VERSION 3.1)

# Define project name
project(Robotronikaruco)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)

find_package(X11 REQUIRED)
add_compile_definitions(WITH_X11)

find_package(CUDAToolkit)
if(CUDAToolkit_FOUND)
    add_compile_definitions(WITH_CUDA)
    message(STATUS "CUDA found, assuming OpenCV was built with CUDA")
endif(CUDAToolkit_FOUND)


# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "Cmake source dir : ${CMAKE_SOURCE_DIR}")
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

MACRO(HEADER_DIRECTORIES return_list)
    FILE(GLOB_RECURSE new_list include/*.h*)
    SET(dir_list "")
    FOREACH(file_path ${new_list})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
        SET(dir_list ${dir_list} ${dir_path})
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
    SET(${return_list} ${dir_list})
ENDMACRO()

HEADER_DIRECTORIES(INCLUDES)

message(STATUS "include directories : ${INCLUDES}")

include_directories(
	${OPENCV_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/include)

file(GLOB_RECURSE SOURCES "source/*.cpp")

add_executable(Robotronikaruco ${SOURCES})



message(STATUS "Sources found : ${SOURCES}")

set_target_properties(Robotronikaruco PROPERTIES CXX_STANDARD 17)
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")

# Link your application with OpenCV libraries

target_link_libraries(Robotronikaruco 
	${OpenCV_LIBS} 
	stdc++fs
	${X11_LIBRARIES}
	config++
)