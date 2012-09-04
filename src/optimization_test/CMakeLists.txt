cmake_minimum_required(VERSION 2.8)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/dm  ${PROJECT_SOURCE_DIR}/src/dmu ${PROJECT_SOURCE_DIR}/src/dmgl ${PROJECT_SOURCE_DIR}/src/wxdmgl ${PROJECT_SOURCE_DIR}/src/mosek/6/tools/platform/linux64x86/h  )
LINK_DIRECTORIES(${PROJECT_BINARY_DIR}/lib )


#find_package(wxWidgets COMPONENTS core base gl REQUIRED)

#include( "${wxWidgets_USE_FILE}" )

#tell CMake to search first in directories listed in CMAKE_MODULE_PATH 
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeScripts)
message("CMAKE_MODULE_PATH (search first) is ${CMAKE_MODULE_PATH}")


SET(wxWidgets_ROOT_DIR /usr/local/bin)
find_package(wxWidgets 2.9 COMPONENTS core base gl REQUIRED)
if(wxWidgets_FOUND)
  message("wxWidgets found successfully.")
  include( ${wxWidgets_USE_FILE} )
else()
  message(FATAL_ERROR "wxWidgets was not found!")
endif()

#if(wxWidgets_PFVERSION LESS 2.9)
#  message("wxWidgets Version ${wxWidgets_PFVERSION}")
#  #message(FATAL_ERROR "wxWidgets is not a high enough version!")
#
#else()
#  message("wxWidgets Version ${wxWidgets_PFVERSION}")
#
#endif()




SET(SRC1  src/eigen_func_test.cpp)
ADD_EXECUTABLE(func_test ${SRC1})


SET(SRCS src/QPsolver.cpp
         src/math_funcs.cpp
         src/main.cpp)
ADD_EXECUTABLE(test1 ${SRCS})
TARGET_LINK_LIBRARIES(test1 ${wxWidgets_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} dm dmu wxdmgl ${PROJECT_SOURCE_DIR}/src/mosek/6/tools/platform/linux64x86/bin/libmosek64.so ${PROJECT_SOURCE_DIR}/src/mosek/6/tools/platform/linux64x86/bin/libmoseknoomp64.so)


SET(SRCS src/wxMain.cpp
         src/BasicGLPane.cpp
         src/DataLogger.cpp
         src/OTDataLogger.cpp
         src/MainFrame.cpp
         src/SimulationThread.cpp
         src/globalVariables.cpp
         src/globalFunctions.cpp
         src/userGraphics.cpp
)
ADD_EXECUTABLE(opt_test ${SRCS})
TARGET_LINK_LIBRARIES(opt_test ${wxWidgets_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} dm dmu wxdmgl ${PROJECT_SOURCE_DIR}/src/mosek/6/tools/platform/linux64x86/bin/libmosek64.so ${PROJECT_SOURCE_DIR}/src/mosek/6/tools/platform/linux64x86/bin/libmoseknoomp64.so)






