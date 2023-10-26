cmake_minimum_required(VERSION 3.0.2)
project(omega_haptic)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

catkin_package(
    INCLUDE_DIRS include
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
	message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential")
endif()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/release/lin-x86_64-gcc)

add_executable(omega_haptic
	src/haptic_ros_driver.cpp
	src/omega_haptic_control.cpp
)

add_dependencies(omega_haptic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(omega_haptic
    -lpthread -ldhd -lrt -ldl -lusb-1.0
    ${catkin_LIBRARIES}
    
) 
