cmake_minimum_required(VERSION 2.8)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/dm  ${PROJECT_SOURCE_DIR}/src/dmu ${PROJECT_SOURCE_DIR}/src/dmgl)
LINK_DIRECTORIES(${PROJECT_BINARY_DIR}/lib)

SET(BIPED_SRCS  flywheel_biped.cpp global.cpp functions_A.cpp functions_B.cpp functions_C.cpp functions_D.cpp)
ADD_EXECUTABLE(flywheelbiped ${BIPED_SRCS})

#SET(FUNC_SRC functions.cpp)
#ADD_LIBRARY(func1 SHARED ${FUNC_SRC})
#SET_TARGET_PROPERTIES(func1 PROPERTIES VERSION 5.0 SOVERSION 1)
#SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

TARGET_LINK_LIBRARIES(flywheelbiped ${GLUT_LIBRARY} ${OPENGL_LIBRARY} dm dmu dmgl)





