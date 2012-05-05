cmake_minimum_required(VERSION 2.8)
SET(DM_SRCS  dm.h dmTime.h 
	dmObject.hpp dmObject.cpp 
	dmSystem.hpp dmSystem.cpp 
	dmArticulation.hpp dmArticulation.cpp 
	dmClosedArticulation.hpp dmClosedArticulation.cpp 
	dmLink.hpp dmLink.cpp 
	dmStaticRootLink.hpp dmStaticRootLink.cpp 
	dmZScrewTxLink.hpp dmZScrewTxLink.cpp 
	dmRigidBody.hpp dmRigidBody.cpp 
	dmMobileBaseLink.hpp dmMobileBaseLink.cpp 
	dmSphericalLink.hpp dmSphericalLink.cpp 
	dmQuaternionLink.hpp dmQuaternionLink.cpp 
	dmMDHLink.hpp dmMDHLink.cpp 
	dmPrismaticLink.hpp dmPrismaticLink.cpp 
	dmRevoluteLink.hpp dmRevoluteLink.cpp 
	dmActuator.hpp dmActuator.cpp 
	dmRevDCMotor.hpp dmRevDCMotor.cpp 
	dmEnvironment.hpp dmEnvironment.cpp 
	dmTreadmill.hpp dmTreadmill.cpp 
	dmForce.hpp dmForce.cpp 
	dmContactModel.hpp dmContactModel.cpp 
	dmSecondaryForce.hpp dmSecondaryForce.cpp 
	dmSecondaryJoint.hpp dmSecondaryJoint.cpp 
	dmSecondarySphericalJoint.hpp dmSecondarySphericalJoint.cpp 
	dmSecondaryPrismaticJoint.hpp dmSecondaryPrismaticJoint.cpp 
	dmSecondaryRevoluteJoint.hpp dmSecondaryRevoluteJoint.cpp 
	dmIntegrator.hpp dmIntegrator.cpp 
	dmIntegEuler.hpp dmIntegEuler.cpp 
	dmIntegRK4.hpp dmIntegRK4.cpp 
	dmIntegRK45.hpp dmIntegRK45.cpp 
	svd_linpack.cpp 
	gldraw.cpp )

ADD_LIBRARY(dm SHARED ${DM_SRCS})
TARGET_LINK_LIBRARIES(dm ${OPENGL_LIBRARY} rt)

SET_TARGET_PROPERTIES(dm PROPERTIES VERSION 5.0 SOVERSION 1)

SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

INSTALL(TARGETS dm 
        LIBRARY DESTINATION dynamechs/lib)
FILE( GLOB DM_HEADERS [^.]*.h [^.]*.hpp )
# MESSAGE(STATUS "dm headers" ${DM_HEADERS})
INSTALL(FILES ${DM_HEADERS} DESTINATION dynamechs/dm)

