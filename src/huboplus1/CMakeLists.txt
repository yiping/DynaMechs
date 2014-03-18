cmake_minimum_required(VERSION 2.8)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    SET(MOSEK_LIB C:/Program\ Files\ \(x86\)/Mosek/6/tools/platform/win32x86/bin/mosek6_0.lib)
    SET(MOSEKNOOMP_LIB C:/Program\ Files\ \(x86\)/Mosek/6/tools/platform/win32x86/bin/moseknoomp6_0.lib)
    SET(MOSEK_INCLUDE_DIR C:/Program\ Files\ \(x86\)/Mosek/6/tools/platform/win32x86/h)
ELSE()
    SET(MOSEK_LIB ${PROJECT_SOURCE_DIR}/src/mosek/6/tools/platform/linux64x86/bin/libmosek64.so)
    SET(MOSEKNOOMP_LIB ${PROJECT_SOURCE_DIR}/src/mosek/6/tools/platform/linux64x86/bin/libmoseknoomp64.so)
    SET(MOSEK_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src/mosek/6/tools/platform/linux64x86/h)
ENDIF()


INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/dm  ${PROJECT_SOURCE_DIR}/src/dmu ${PROJECT_SOURCE_DIR}/src/dmgl ${PROJECT_SOURCE_DIR}/src/wxdmgl gui_src ctrl_src ${MOSEK_INCLUDE_DIR} )

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
        message("huboplus1: wxWidgets found successfully :) ")
		message(STATUS "wxWidgets Libraries: " ${wxWidgets_LIBRARIES})
        include( ${wxWidgets_USE_FILE} )
    else()
        message(FATAL_ERROR "wxWidgets was not found! :( ")
    endif()


ENDIF()



SET(SRCS gui_src/wxMain.cpp
         gui_src/BasicGLPane.cpp
         gui_src/MainFrame.cpp
         gui_src/SimulationThread.cpp
         gui_src/globalVariables.cpp
         gui_src/userGraphics.cpp
         ctrl_src/globalFunctions.cpp
         ctrl_src/math_funcs.cpp
         ctrl_src/DataLogger.cpp
         ctrl_src/TaskSpaceControllerA.cpp
         ctrl_src/TaskSpaceControllerConic.cpp
         ctrl_src/HumanoidController.cpp
         ctrl_src/HumanoidControllerStateMachine.cpp
         ctrl_src/HuboController.cpp
         ctrl_src/HuboBalanceController.cpp
)
ADD_EXECUTABLE(hbp1 ${SRCS})
TARGET_LINK_LIBRARIES(hbp1 ${wxWidgets_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} dm dmu wxdmgl ${MOSEK_LIB} ${MOSEKNOOMP_LIB})






