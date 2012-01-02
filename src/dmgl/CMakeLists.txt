cmake_minimum_required(VERSION 2.8)
SET(DMGL_SRCS  	dmGL.h 
	dmGLMouse.hpp dmGLMouse.cpp 
	dmGLCamera.hpp dmGLCamera.cpp 
	dmGLFlyCamera.hpp dmGLFlyCamera.cpp 
	dmGLPolarCamera.hpp dmGLPolarCamera.cpp 
	dmGLPolarCamera_zup.hpp dmGLPolarCamera_zup.cpp 
	dmGLPolarCamera_yup.hpp	dmGLPolarCamera_yup.cpp	)

ADD_LIBRARY(dmgl SHARED ${DMGL_SRCS})
TARGET_LINK_LIBRARIES(dmgl ${GLUT_LIBRARY} ${OPENGL_LIBRARY} dm dmu)

SET_TARGET_PROPERTIES(dmgl PROPERTIES VERSION 4.0 SOVERSION 1)

SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
