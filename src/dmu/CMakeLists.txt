cmake_minimum_required(VERSION 2.8)
SET(DMU_SRCS  	dmu.h dmUtils.cpp 
	dmLoadFile.h dmLoadFile.cpp 
	dmLoadFile_dm.cpp 
	dmLoadFile_dm40.cpp 
	dmLoadFile_dm30.cpp 
	dmLoadFile_dm21.cpp 
	dmLoadFile_dm203.cpp 
	dmLoadFile_env.cpp 
	glLoadModels.h glLoadModels.cpp )

ADD_LIBRARY(dmu SHARED ${DMU_SRCS})
TARGET_LINK_LIBRARIES(dmu ${GLUT_LIBRARY} ${OPENGL_LIBRARY} dm)

SET_TARGET_PROPERTIES(dmu PROPERTIES VERSION 4.0 SOVERSION 1)

SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
