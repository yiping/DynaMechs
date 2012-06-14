cmake_minimum_required(VERSION 2.8)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/dm  ${PROJECT_SOURCE_DIR}/src/dmu ${PROJECT_SOURCE_DIR}/src/dmgl ${PROJECT_SOURCE_DIR}/src/wxdmgl)
LINK_DIRECTORIES(${PROJECT_BINARY_DIR}/lib)

# It was noticed that when using MinGW gcc it is essential that 'core' is mentioned before 'base'.
find_package(wxWidgets COMPONENTS core base gl REQUIRED)

include( "${wxWidgets_USE_FILE}" )

SET(SRCS global.cpp main.cpp userControl.cpp)
ADD_EXECUTABLE(DMViewer ${SRCS})


TARGET_LINK_LIBRARIES(DMViewer ${wxWidgets_LIBRARIES} ${OPENGL_LIBRARY} dm dmu wxdmgl)





