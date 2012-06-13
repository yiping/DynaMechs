
cmake_minimum_required(VERSION 2.8)

SET(WXDMGL_SRCS wxDMGLMouse.cpp 
              wxDMGLCamera.cpp
              wxDMGLPolarCamera.cpp 
              wxDMGLPolarCamera_zup.cpp 
              wxDMGLPolarCamera_yup.cpp 
              wxDMGLFlyCamera.cpp)
ADD_LIBRARY(wxdmgl SHARED ${WXDMGL_SRCS})
TARGET_LINK_LIBRARIES(wxdmgl ${OPENGL_LIBRARY})
SET_TARGET_PROPERTIES(wxdmgl PROPERTIES VERSION 5.0 SOVERSION 1)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
