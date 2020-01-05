cmake_minimum_required(VERSION 3.12)

project(kinematics_sovle)


set(CMAKE_CXX_STANDARD 17)


if(UNIX)
	add_definitions(-DUNIX)
	find_package(aris REQUIRED PATHS /usr/aris)
endif(UNIX)
IF(WIN32)
	find_package(aris REQUIRED PATHS C:/aris/aris-1.5.0)
endif(WIN32)
include_directories(${aris_INCLUDE_DIRS})

add_executable(test src/main.cpp src/kinematic.h src/kinematic.cpp src/plan.h src/plan.cpp src/json.hpp)
target_link_libraries(test ${aris_LIBRARIES} C:/Users/DELL1/Desktop/self_test/lib/MCDLL_NET.lib)



