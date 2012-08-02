cmake_minimum_required(VERSION 2.8)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/dm  ${PROJECT_SOURCE_DIR}/src/dmu ${PROJECT_SOURCE_DIR}/src/dmgl ${PROJECT_SOURCE_DIR}/src/wxdmgl ${PROJECT_SOURCE_DIR}/src/mosek/6/tools/platform/linux64x86/h )
LINK_DIRECTORIES(${PROJECT_BINARY_DIR}/lib )


find_package(wxWidgets COMPONENTS core base gl REQUIRED)

include( "${wxWidgets_USE_FILE}" )

SET(SRCS  srcs/main.cpp
		  srcs/BasicGLPane.cpp
		  srcs/ContactMainFrame.cpp
		  srcs/SimulationThread.cpp
		  srcs/userGraphics.cpp
		  srcs/globalFunctions.cpp
          srcs/globalVariables.cpp
		  srcs/DataLogger.cpp
		  srcs/ContactDemoDataLogger.cpp)
ADD_EXECUTABLE(contact_demo ${SRCS})


TARGET_LINK_LIBRARIES(contact_demo ${wxWidgets_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} dm dmu wxdmgl ${PROJECT_SOURCE_DIR}/src/mosek/6/tools/platform/linux64x86/bin/libmosek64.so ${PROJECT_SOURCE_DIR}/src/mosek/6/tools/platform/linux64x86/bin/libmoseknoomp64.so)






