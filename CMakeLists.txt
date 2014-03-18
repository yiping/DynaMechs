cmake_minimum_required(VERSION 2.8)
PROJECT(DynaMechs)
MESSAGE(STATUS "This is BINARY dir " ${PROJECT_BINARY_DIR})
MESSAGE(STATUS "This is SOURCE dir " ${PROJECT_SOURCE_DIR})
MESSAGE(STATUS "CMAKE_ROOT is " ${CMAKE_ROOT})
MESSAGE(STATUS "CMAKE_SYSTEM_NAME is " ${CMAKE_SYSTEM_NAME})

SET(CMAKE_MODULE_PATH "${CMAKE_ROOT}/Modules;${CMAKE_MODULE_PATH}")
message(STATUS "CMAKE_MODULE_PATH is " ${CMAKE_MODULE_PATH} )

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
MESSAGE(STATUS "Adding Mac hooks (tested with OSX 10.7)")
SET(CMAKE_CXX_FLAGS "-isysroot /Developer/SDKs/MacOSX10.6.sdk")
SET(CMAKE_SHARED_LINKER_FLAGS "-lglut -L/usr/X11/lib")
SET(CMAKE_EXE_LINKER_FLAGS "-lglut -L/usr/X11/lib")
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


MESSAGE(STATUS "c compiler is: " ${CMAKE_C_COMPILER} )
MESSAGE(STATUS "c++ compiler is: " ${CMAKE_CXX_COMPILER} )

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	#tell CMake to search first in directories listed in CMAKE_MODULE_PATH
	SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeScripts)
	message(STATUS "CMAKE_MODULE_PATH (search first) is ${CMAKE_MODULE_PATH}")
    SET(GLUT_ROOT_PATH ${PROJECT_SOURCE_DIR}/glut)
ENDIF()

# /usr/share/cmake-2.8/Modules
find_package(GLUT)
find_package(OpenGL)

#override opengl path, using ATI implementation
#set(OPENGL_LIBRARIES "/usr/lib/fglrx")

MESSAGE(STATUS "GLUT headers path: " ${GLUT_INCLUDE_DIR})
MESSAGE(STATUS "GLUT library path: " ${GLUT_LIBRARIES})

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    MESSAGE(STATUS "openGL library : " ${OPENGL_gl_LIBRARY})
    MESSAGE(STATUS "glu library : " ${OPENGL_glu_LIBRARY})
ELSE()
    MESSAGE(STATUS "OpenGL headers path: " ${OPENGL_INCLUDE_DIR})
    MESSAGE(STATUS "OpenGL library path: " ${OPENGL_LIBRARIES})
ENDIF()


MESSAGE(STATUS "CMake build type: " ${CMAKE_BUILD_TYPE})

# Eigen is a header-only library, so you do not need to specify any linking

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    SET(EIGEN_INCLUDE_DIR "$ENV{EIGEN_INCLUDE_DIR}")
    IF( NOT EIGEN_INCLUDE_DIR )
        MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN_INCLUDE_DIR to the include directory of your Eigen3 installation.")
    ENDIF(NOT EIGEN_INCLUDE_DIR)
    INCLUDE_DIRECTORIES ( "${EIGEN_INCLUDE_DIR}" )
    MESSAGE(STATUS "EIGEN_INCLUDE_DIR: " ${EIGEN_INCLUDE_DIR})
ELSE()
    find_package(Eigen REQUIRED)
    include_directories(${Eigen_INCLUDE_DIRS})
    MESSAGE(STATUS "Eigen3 headers path: " ${Eigen_INCLUDE_DIRS})
ENDIF()



ADD_SUBDIRECTORY(src/dm build/bin_dm)
ADD_SUBDIRECTORY(src/dmu build/bin_dmu)
ADD_SUBDIRECTORY(src/dmgl build/bin_dmgl)
ADD_SUBDIRECTORY(src/wxdmgl build/bin_wxdmgl)
#ADD_SUBDIRECTORY(src/testdm bin_testdm)
#ADD_SUBDIRECTORY(src/aquarobot bin_aquarobot)
#ADD_SUBDIRECTORY(src/xan_viewer bin_xan_viewer)
#ADD_SUBDIRECTORY(src/kurmet bin_kurmet)
#ADD_SUBDIRECTORY(src/windmill bin_windmill)
#ADD_SUBDIRECTORY(src/flywheel_biped bin_flywheel_biped)
#ADD_SUBDIRECTORY(src/wxDMV bin_wxDMV)
ADD_SUBDIRECTORY(src/humanoid build/bin_humanoid)
#ADD_SUBDIRECTORY(src/contact_demo bin_contact_demo)
#ADD_SUBDIRECTORY(src/optimization_test build/bin_optimization_test)
ADD_SUBDIRECTORY(src/showRobot build/bin_showRobot)
ADD_SUBDIRECTORY(src/huboplus1 build/bin_huboplus1)
#ADD_SUBDIRECTORY(src/walking build/bin_walk)

MESSAGE(STATUS "CMAKE_INSTALL_PREFIX is: " ${CMAKE_INSTALL_PREFIX})

