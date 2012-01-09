cmake_minimum_required(VERSION 2.8)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/dm 
                    ${PROJECT_SOURCE_DIR}/src/dmu 
                    ${PROJECT_SOURCE_DIR}/src/dmgl
                    ${PROJECT_SOURCE_DIR}/src/aquarobot)
LINK_DIRECTORIES(${PROJECT_BINARY_DIR}/lib )

SET(AQUAFUNCS	Vector.cpp 
	EulerAng.cpp 
	Posture.cpp 
	MatrixMy.cpp 
	LegLink.cpp 
	Leg.cpp 
	Body.cpp 
	Robot.cpp 
	GaitAlgorithm.cpp)


ADD_LIBRARY(aquafuncs SHARED ${AQUAFUNCS})

SET(AQUA_SRC aquarobot.cpp )
ADD_EXECUTABLE(aquarobot ${AQUA_SRC})
TARGET_LINK_LIBRARIES(aquarobot ${GLUT_LIBRARY} ${OPENGL_LIBRARY} dm dmu dmgl aquafuncs)





