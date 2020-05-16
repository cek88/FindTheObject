cmake_minimum_required(VERSION 3.2.1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#find_package(PCL 1.7 REQUIRED)

project(FindTheObject)

#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})


#add_library(my_lib Parser/Parser.cpp)
#target_include_directories(my_lib PUBLIC include/my_lib_inc)

add_executable(FindTheObject main.cpp Parser/Parser.cpp Tools/randomGrabber.cpp Tools/evalRansac.cpp Tools/icpEvaluator) #Point_Registrator/Registrator.cpp)#  Plotter/Plotter.cpp)
#target_link_libraries (FindTheObject ${PCL_LIBRARIES})
