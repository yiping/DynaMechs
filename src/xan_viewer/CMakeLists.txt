cmake_minimum_required(VERSION 2.8)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/dm  ${PROJECT_SOURCE_DIR}/src/dmu ${PROJECT_SOURCE_DIR}/src/dmgl )
LINK_DIRECTORIES(${PROJECT_BINARY_DIR}/lib  ${CMAKE_INSTALL_PREFIX}/dynamechs/lib)

SET(XAN_VIEWER_SRC  xan_viewer.cpp )
ADD_EXECUTABLE(xan_viewer ${XAN_VIEWER_SRC})
TARGET_LINK_LIBRARIES(xan_viewer ${GLUT_LIBRARY} ${OPENGL_LIBRARY} dm dmu dmgl)





INSTALL(TARGETS xan_viewer
                 RUNTIME DESTINATION bin)
set_target_properties(xan_viewer PROPERTIES 
                      INSTALL_RPATH_USE_LINK_PATH TRUE)

#set_target_properties(xan_viewer PROPERTIES INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)
#set_property(TARGET xan_viewer PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)
