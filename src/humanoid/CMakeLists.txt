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

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/dm  ${PROJECT_SOURCE_DIR}/src/dmu ${PROJECT_SOURCE_DIR}/src/dmgl ${PROJECT_SOURCE_DIR}/src/wxdmgl ${MOSEK_INCLUDE_DIR} ${PROJECT_SOURCE_DIR}/src/humanoid/h )
LINK_DIRECTORIES(${PROJECT_BINARY_DIR}/lib )


IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    # It was noticed that when using MinGW gcc it is essential that 'core' is mentioned before 'base'
    find_package(wxWidgets COMPONENTS core base gl REQUIRED)
    include( "${wxWidgets_USE_FILE}" )
ELSE()

    find_package(wxWidgets 2.9 COMPONENTS core base gl REQUIRED)
    if(wxWidgets_FOUND)
        message("humaoind: wxWidgets found successfully :)")
		message(STATUS "wxWidgets Libraries: " ${wxWidgets_LIBRARIES})
        include( ${wxWidgets_USE_FILE} )
    else()
        message(FATAL_ERROR "wxWidgets was not found!")
    endif()


ENDIF()






SET(SRCS  cpp/humanoidWx.cpp
		  cpp/StateMachine.cpp
		  cpp/BalanceDemoStateMachine.cpp
		  cpp/JumpingStateMachine.cpp
		  cpp/BasicGLPane.cpp
		  cpp/HumanoidMainFrame.cpp
		  cpp/Humanoid.cpp
          cpp/HumanoidController.cpp 
		  cpp/HumanoidDataLogger.cpp
		  cpp/HumanoidStateMachineController.cpp
		  cpp/SimulationThread.cpp
          cpp/CubicSpline.cpp
          cpp/CubicSplineTrajectory.cpp
          cpp/DataLogger.cpp
          cpp/GlobalDefines.cpp
		  cpp/GlobalFunctions.cpp
          cpp/TaskSpaceController.cpp
		  cpp/TaskSpaceControllerA.cpp
		  cpp/TaskSpaceControllerB.cpp
		  cpp/wxBoxSlider.cpp
          cpp/wxNumericTextCtrl.cpp)
ADD_EXECUTABLE(humanoid ${SRCS})


TARGET_LINK_LIBRARIES(humanoid ${wxWidgets_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} dm dmu wxdmgl ${MOSEK_LIB} ${MOSEKNOOMP_LIB})






