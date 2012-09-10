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
MESSAGE(STATUS "CMake build type: " ${CMAKE_BUILD_TYPE})

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
# Eigen is a header-only library, so you do not need to specify any linking
MESSAGE(STATUS "Eigen headers path: " ${Eigen_INCLUDE_DIRS})

ADD_SUBDIRECTORY(src/dm bin_dm)
ADD_SUBDIRECTORY(src/dmu bin_dmu)
ADD_SUBDIRECTORY(src/dmgl bin_dmgl)
ADD_SUBDIRECTORY(src/wxdmgl bin_wxdmgl)
ADD_SUBDIRECTORY(src/testdm bin_testdm)
#ADD_SUBDIRECTORY(src/aquarobot bin_aquarobot)
#ADD_SUBDIRECTORY(src/xan_viewer bin_xan_viewer)
#ADD_SUBDIRECTORY(src/kurmet bin_kurmet)
#ADD_SUBDIRECTORY(src/windmill bin_windmill)
#ADD_SUBDIRECTORY(src/flywheel_biped bin_flywheel_biped)
ADD_SUBDIRECTORY(src/wxDMV bin_wxDMV)
ADD_SUBDIRECTORY(src/humanoid bin_humanoid)
#ADD_SUBDIRECTORY(src/contact_demo bin_contact_demo)
ADD_SUBDIRECTORY(src/optimization_test bin_optimization_test)

MESSAGE(STATUS "CMAKE_INSTALL_PREFIX is: " ${CMAKE_INSTALL_PREFIX})

