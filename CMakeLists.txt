cmake_minimum_required(VERSION 2.8)
PROJECT(DynaMechs)
MESSAGE(STATUS "This is BINARY dir " ${PROJECT_BINARY_DIR})
MESSAGE(STATUS "This is SOURCE dir " ${PROJECT_SOURCE_DIR})
MESSAGE(STATUS "CMAKE_ROOT is" ${CMAKE_ROOT})

SET(CMAKE_CXX_FLAGS "-isysroot /Developer/SDKs/MacOSX10.6.sdk")
MESSAGE(STATUS "CMAKE_CXX_FLAGS is " ${CMAKE_CXX_FLAGS})
# /usr/share/cmake-2.8/Modules
find_package(GLUT)
find_package(OpenGL)

ADD_SUBDIRECTORY(src/dm bin_dm)
ADD_SUBDIRECTORY(src/dmu bin_dmu)
ADD_SUBDIRECTORY(src/dmgl bin_dmgl)
ADD_SUBDIRECTORY(src/testdm bin_testdm)

