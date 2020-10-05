cmake_minimum_required(VERSION 3.1)

project (IMU_GPS)

# set( CMAKE_BUILD_TYPE "Release" )

include_directories("/usr/include/eigen3" )

add_executable(IMU_GPS 
	IMU_GPS_main.cpp 
	wmwSP.cpp 
	wmwSP.h
	global_var.h
	global_var.cpp
	)

