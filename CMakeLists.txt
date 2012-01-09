cmake_minimum_required(VERSION 2.8)
PROJECT(DynaMechs)
MESSAGE(STATUS "This is BINARY dir " ${PROJECT_BINARY_DIR})
MESSAGE(STATUS "This is SOURCE dir " ${PROJECT_SOURCE_DIR})
MESSAGE(STATUS "CMAKE_ROOT is" ${CMAKE_ROOT})
MESSAGE(STATUS "CMAKE_SYSTEM_NAME is " ${CMAKE_SYSTEM_NAME})

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
MESSAGE(STATUS "Adding Mac hooks (tested with OSX 10.7)")
SET(CMAKE_CXX_FLAGS "-isysroot /Developer/SDKs/MacOSX10.6.sdk")
SET(CMAKE_SHARED_LINKER_FLAGS "-lglut -L/usr/X11/lib")
SET(CMAKE_EXE_LINKER_FLAGS "-lglut -L/usr/X11/lib")
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


# /usr/share/cmake-2.8/Modules
find_package(GLUT)
find_package(OpenGL)
MESSAGE(STATUS "OpenGL library path: " ${OPENGL_LIBRARIES})
MESSAGE(STATUS "GLUT library path: " ${GLUT_LIBRARIES})
MESSAGE(STATUS "OpenGL headers path: " ${OPENGL_INCLUDE_DIR})
MESSAGE(STATUS "GLUT headers path: " ${GLUT_INCLUDE_DIR})

ADD_SUBDIRECTORY(src/dm bin_dm)
ADD_SUBDIRECTORY(src/dmu bin_dmu)
ADD_SUBDIRECTORY(src/dmgl bin_dmgl)
ADD_SUBDIRECTORY(src/testdm bin_testdm)
ADD_SUBDIRECTORY(src/aquarobot bin_aquarobot)
ADD_SUBDIRECTORY(src/xan_viewer bin_xan_viewer)

MESSAGE(STATUS "CMAKE_INSTALL_PREFIX is: " ${CMAKE_INSTALL_PREFIX})

