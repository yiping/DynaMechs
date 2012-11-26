cmake_minimum_required(VERSION 2.8)


INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/dm  ${PROJECT_SOURCE_DIR}/src/dmu ${PROJECT_SOURCE_DIR}/src/dmgl ${PROJECT_SOURCE_DIR}/src/wxdmgl ${MOSEK_INCLUDE_DIR} )

LINK_DIRECTORIES(${PROJECT_BINARY_DIR}/lib )

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    # It was noticed that when using MinGW gcc it is essential that 'core' is mentioned before 'base'
    find_package(wxWidgets COMPONENTS core base gl REQUIRED)
    include( "${wxWidgets_USE_FILE}" )
ELSE()
    ##tell CMake to search first in directories listed in CMAKE_MODULE_PATH 
    #SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeScripts)
    #message("CMAKE_MODULE_PATH (search first) is ${CMAKE_MODULE_PATH}")

    #SET(wxWidgets_ROOT_DIR /usr/local/bin)
    find_package(wxWidgets 2.9 COMPONENTS core base gl REQUIRED)
    if(wxWidgets_FOUND)
        message("optimization_test: wxWidgets found successfully :) ")
		message(STATUS "wxWidgets Libraries: " ${wxWidgets_LIBRARIES})
        include( ${wxWidgets_USE_FILE} )
    else()
        message(FATAL_ERROR "wxWidgets was not found! :( ")
    endif()


ENDIF()



SET(SRCS src/wxMain.cpp
         src/BasicGLPane.cpp
         src/MainFrame.cpp
         src/SimulationThread.cpp
         src/globalVariables.cpp
         src/globalFunctions.cpp
         src/userGraphics.cpp
         src/math_funcs.cpp
)
ADD_EXECUTABLE(showRobot ${SRCS})
TARGET_LINK_LIBRARIES(showRobot ${wxWidgets_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} dm dmu wxdmgl )






