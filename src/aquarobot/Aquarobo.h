// **************************************************************************
// FILENAME: Aquarobo.h
// PURPOSE:  THIS FILE REPRESENTS AQUAROBOT'S CONSTANT PARAMETERS 
// AUTHOR:   Kenji Suzuki
// DATE:     18 Feb 1993
// COMMENT:  unit: length=[cm], time=[sec], angle=[rad], 
// UPDATE:   15 June 1993 
// **************************************************************************

#ifndef AQUAROBO_H
#define AQUAROBO_H

#include <Gait.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define PI M_PI
#define DPI (2.0*PI)
#define HPI (PI/2.0)
#define DR (PI/180.0)
#define RD (180.0/PI)
#define D2R (PI/180.0)
#define R2D (180.0/PI)

#define ROOT3 1.732050807568877193176604123436845839023590087890625
#define EQU_TRIANGLE (3.0/2.0)

#define LEG  6
#define LEG1 0
#define LEG2 1
#define LEG3 2
#define LEG4 3
#define LEG5 4
#define LEG6 5

#define CB    0
#define HIP   1
#define KNEE1 2
#define KNEE2 3
#define FOOT  4

#define VJOINT 4
#define JOINT  5
#define JOINT0 0
#define JOINT1 1
#define JOINT2 2
#define JOINT3 3
#define JOINT4 4

// link length
#define LINK0  37.5 /* Center of the Body to HIP joint */
#define LINK1  20.0 /* HIP to KNEE1 joint              */
#define LINK2  52.0 /* KNEE1 to KNEE2 joint            */
#define LINK3 102.0 /* KNEE2 to FOOT(ANKLE) joint      */
#define LINK4   3.0 /* FOOT to SOLE                    */

// square of link length
#define LINK02  (37.5* 37.5)
#define LINK12  (20.0* 20.0)
#define LINK22  (52.0* 52.0)
#define LINK32 (102.0*102.0)
#define LINK42  ( 3.0*  3.0)


#define MAXMIN 2

#define TWO_DIM   2
#define THREE_DIM 3
#define XY        2
#define XYZ       3
#define EULER     3
#define XW 0
#define YW 1
#define ZW 2
#define XB 0
#define YB 1
#define ZB 2
#define X 0
#define Y 1
#define Z 2

// Joint angles when the robot is in RESET posture.
#define J1ANGLE_RESET (   0.00*DR) /* HIP   */
#define J2ANGLE_RESET (  67.38*DR) /* KNEE1 */
#define J3ANGLE_RESET (-157.38*DR) /* KNEE2 */
#define J4ANGLE_RESET (   0.00*DR) /* ANKLE */

// Joint angles when the robot is in START posture.
#define J1ANGLE_START (   0.00*DR) /* HIP   */
#define J2ANGLE_START (   8.21*DR) /* KNEE1 */
#define J3ANGLE_START ( -98.21*DR) /* KNEE2 */
#define J4ANGLE_START (   0.00*DR) /* ANKLE */

// Minimum joint angle limit.
#define J0ANGLE_MIN   (   0.00*DR)
#define J1ANGLE_MIN   ( -60.00*DR) /* HIP   */
#define J2ANGLE_MIN   (-105.62*DR) /* KNEE1 */
#define J3ANGLE_MIN   (-157.38*DR) /* KNEE2 */
#define J4ANGLE_MIN   ( -22.00*DR) /* ANKLE */

// Maxmum joint angle limit.
#define J0ANGLE_MAX   ( 300.00*DR)
#define J1ANGLE_MAX   (  60.00*DR) /* HIP   */
#define J2ANGLE_MAX   (  74.38*DR) /* KNEE1 */
#define J3ANGLE_MAX   (  22.62*DR) /* KNEE2 */
#define J4ANGLE_MAX   (  22.00*DR) /* ANKLE */

// distance from CB to FOOT when robot is in START posture.
#define STANCE       108.97
#define STRIDE       120.0
#define FOOT_HEIGHT   60.0 //20.0
#define BODY_HEIGHT   94.57
#define BODY_SPEED     5.0 /* [cm/sec] */
#define R_CWV         25.0 /* radius of cwv [cm]  (32cm) */
#define D_CWV         (2.0*R_CWV)

// These are for array indexes
// #define ORIENTATION 3
#define AZIMUTH   0
#define ELEVATION 1
#define ROLL      2

#define YAW   0
#define PITCH 1
#define ROLL  2

// #define AZIMUTH   3
// #define ELEVATION 4
// #define ROLL      5
// #define YAW   3
// #define PITCH 4
// #define ROLL  5

#define FINE    5.0
#define DS_MAX  5.0
#define DS_INIT 1.0
// #define SCONST (DS_MAX/(90.0*90.0))

#define TP0   0
#define BODY0 1
#define TP1   2
#define BODY1 3


#define MAX_TRANSFER_B_FOOT_VEL 25.0 // cm/sec.  Previous value: 50.0


#define LONG_TIME 100.0


#if 0
/* came from mml.h(Dr.Kanayama's project)
#define PI              3.14159265358979323846
#define DPI             6.28318530717958647692    // PI * 2
#define RAD            57.29577951308232087684    // 180/PI
#define HPI             1.57079632679489661923    // PI/2  
#define PI34            2.35619449019234492885    // 3PI/4
#define PI4             0.78539816339744830962    // PI/4     	
#define NEGATIVE_SPEED -1.0
*/
#endif

#define aqmin(a,b) ((a)<(b)?(a):(b))
#define aqmax(a,b) ((a)>(b)?(a):(b))
#define sqr(xc)  ((xc)*(xc))

#if 0
/*
#define CUBE(x) ((x)*(x)*(x))
#define SQR(xc) ( (xc) * (xc) )
#define sqr(xc) ( (xc) * (xc) )
#define EU_DIS(x1,y1,x2,y2) (sqrt(((x1)-(x2))*((x1)-(x2))+((y1)-(y2))*((y1)-(y2))))
#define DIST(x1,y1,x2,y2) (fabs((x1)-(x2))+fabs((y1)-(y2)))
#define SQRT(xc) (sqrt(1. + (xc)*(xc)))
#define PAR_LN(x,c) (0.5*(x)*SQRT((x)/(c))+0.5/(c)*log((x)/(c)+SQRT((x)/(c))))
#define r2d(r) ((r) * RAD)
#define d2r(d) ((d) / RAD)
*/
#endif


#endif

// EOF
