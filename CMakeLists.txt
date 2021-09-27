#Specify the version being used as well as the language
cmake_minimum_required(VERSION 2.8.11)

#Name your project here
project(muse-scan)

#set the module directory
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

#find the STIM library
find_package(STIM)

#if the STIM library isn't found, download it
if(NOT STIM_INCLUDE_DIRS)
	file(REMOVE_RECURSE ${CMAKE_BINARY_DIR}/stimlib)	#remove the stimlib directory if it exists
	set(STIM_GIT "https://git.stim.ee.uh.edu/codebase/stimlib.git")
	execute_process(COMMAND git clone --depth 1 ${STIM_GIT} WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
	set(STIM_INCLUDE_DIRS "${CMAKE_BINARY_DIR}/stimlib" CACHE TYPE PATH)
endif(NOT STIM_INCLUDE_DIRS)

#find the pthreads package
find_package(Threads)

#find the X11 package
find_package(X11)

#find the A3200 package
find_package(A3200)

#find OpenCV
find_package(OpenCV REQUIRED)
add_definitions(-DUSING_OPENCV)

#build the executable in the binary directory on MS Visual Studio
if ( MSVC )
	SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${OUTPUT_DIRECTORY}")
	SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${OUTPUT_DIRECTORY}")
endif ( MSVC )

include_directories(
					${STIM_INCLUDE_DIRS}
					${A3200_INCLUDE_DIRS}
					${OpenCV_INCLUDE_DIRS}
					)

#Assign source files to the appropriate variables
file(GLOB TSI_SRC_H "source/tsi/*.h")
file(GLOB TSI_SRC_C "source/tsi/*.c")
file(GLOB TSI_SRC_CPP "source/tsi/*.cpp")
file(GLOB FMEASURE_SRC_H "source/metric/*.h")
file(GLOB FMEASURE_SRC_CPP "source/metric/*.cpp")
file(GLOB A3200_SRC_H "source/a3200/*.h")
file(GLOB A3200_SRC_CPP "source/a3200/*.cpp")
file(GLOB MUSE_SRC "source/*.cpp")
file(GLOB MUSE_H "source/*.h")

#create MUSE executable file
add_executable(muse-scan
						${TSI_SRC_H}
						${TSI_SRC_C}
						${TSI_SRC_CPP}
						${FMEASURE_SRC_H}
						${FMEASURE_SRC_CPP}
						${A3200_SRC_H}
						${A3200_SRC_CPP}
						${MUSE_SRC}
						${MUSE_H}
						)
target_link_libraries(muse-scan
						${CMAKE_THREAD_LIBS_INIT}
						${X11_LIBRARIES}
						${A3200_LIBRARY}
						${OpenCV_LIBS}
)					
