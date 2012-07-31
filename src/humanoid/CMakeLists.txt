cmake_minimum_required(VERSION 2.8)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/dm  ${PROJECT_SOURCE_DIR}/src/dmu ${PROJECT_SOURCE_DIR}/src/dmgl ${PROJECT_SOURCE_DIR}/src/wxdmgl ${PROJECT_SOURCE_DIR}/src/mosek/6/tools/platform/linux64x86/h ${PROJECT_SOURCE_DIR}/src/humanoid/h)
LINK_DIRECTORIES(${PROJECT_BINARY_DIR}/lib )


find_package(wxWidgets COMPONENTS core base gl REQUIRED)

include( "${wxWidgets_USE_FILE}" )

SET(SRCS  cpp/humanoidWx.cpp
		  cpp/StateMachine.cpp
		  cpp/BalanceDemoStateMachine.cpp
		  cpp/JumpingStateMachine.cpp
		  cpp/BasicGLPane.cpp
		  cpp/MainFrame.cpp
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
		  cpp/wxBoxSlider.cpp
          cpp/wxNumericTextCtrl.cpp)
ADD_EXECUTABLE(humanoid ${SRCS})


TARGET_LINK_LIBRARIES(humanoid ${wxWidgets_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} dm dmu wxdmgl ${PROJECT_SOURCE_DIR}/src/mosek/6/tools/platform/linux64x86/bin/libmosek64.so ${PROJECT_SOURCE_DIR}/src/mosek/6/tools/platform/linux64x86/bin/libmoseknoomp64.so)






