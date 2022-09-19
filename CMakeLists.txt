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
#define location of nvmmapi and argus in jetson
#set(MMAPI_DIR "/usr/src/jetson_multimedia_api")

#set(ARGUS_DIR "${MMAPI_DIR}/argus")
#set(ARGUS_INCLUDE_DIR "${ARGUS_DIR}/")
#set(ARGUS_UTILS_DIR "${ARGUS_DIR}/samples/utils")

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
	message("Jetson Multimedia API found, enabling Argus, OpenGL, EGL, X11 and CUDA. WITH_ARGUS defined.")
	#find_package(Argus REQUIRED)
	find_package(OpenGLES REQUIRED)
	find_package(EGL REQUIRED)
	find_package(CUDA REQUIRED)
	find_package(PkgConfig REQUIRED)
	pkg_check_modules(GTK3 REQUIRED gtk+-3.0)
	pkg_check_modules(Cairo REQUIRED cairo)

	#add_compile_definitions(WITH_ARGUS)
	add_definitions(${GTK3_CFLAGS_OTHER})
	#add_definitions(${NVMMAPI_DEFINITIONS})

	include_directories(
		#${ARGUS_INCLUDE_DIR}
		${EGL_INCLUDE_DIR}
		${OPENGLES_INCLUDE_DIR}
		#${ARGUS_UTILS_DIR}
    	${GTK3_INCLUDE_DIRS})


	#file(GLOB ARG_UTILS_SOURCES "${ARGUS_UTILS_DIR}/*.cpp")
	#file(GLOB ARG_UTILS_SOURCES_GTK "${ARGUS_UTILS_DIR}/gtk/*.cpp")
	#file(GLOB_RECURSE NVMM_SOURCES "${MMAPI_DIR}/samples/common/*.cpp")
	#list(APPEND ARG_UTILS_SOURCES ";${ARG_UTILS_SOURCES_GTK}")
	#set(ARG_UTILS_SOURCES
    #    ${ARG_UTILS_SOURCES}
    #    ${ARGUS_UTILS_DIR}/nvmmapi/NvNativeBuffer.cpp
    #    )
	#message(STATUS "ArgusUtils Sources found : ${ARG_UTILS_SOURCES}")
else(NVMMAPI_FOUND)
	message("Jetson Multimedia API was NOT found. If you are on a jetson, this is bad. Otherwise just don't care")
endif(NVMMAPI_FOUND)

include_directories(
	${OPENCV_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/include)

file(GLOB_RECURSE SOURCES "source/*.cpp")
if(NVMMAPI_FOUND)
	list(APPEND SOURCES ";${ARG_UTILS_SOURCES}")
endif(NVMMAPI_FOUND)

if(NVMMAPI_FOUND)
	cuda_add_executable(Robotronikaruco ${SOURCES})
else(NVMMAPI_FOUND)
	add_executable(Robotronikaruco ${SOURCES})
endif(NVMMAPI_FOUND)



message(STATUS "Sources found : ${SOURCES}")

set_target_properties(Robotronikaruco PROPERTIES CXX_STANDARD 17)

# Link your application with OpenCV libraries

target_link_libraries(Robotronikaruco 
	${OpenCV_LIBS} 
	stdc++fs
	${X11_LIBRARIES}
	config++
)

message(STATUS "Cuda libs : ${CUDA_LIBRARIES}")

if(NVMMAPI_FOUND)
	target_link_libraries(Robotronikaruco
		${ARGUS_LIBRARIES}
		${OPENGLES_LIBRARIES}
		${EGL_LIBRARIES}
		pthread
    	${GTK3_LIBRARIES}
        ${NVMMAPI_LIBRARIES}
        ${CUDA_LIBRARIES}
		cuda
	)
	include(CPack)
else(NVMMAPI_FOUND)
	
endif(NVMMAPI_FOUND)