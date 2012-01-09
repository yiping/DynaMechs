// *************************************************************************
// FILENAME: GaitAlgorithm.h
// AUTHOR:   Kenji Suzuki
// COMMENT:
// UPDATE:   September 5, 1993 by Kenji Suzuki
// *************************************************************************
#ifndef GAITALGORITHM_H
#define GAITALGORITHM_H

#include "Robot.hpp"

#define CYCLOID  1
#define ELLIPSE  2
#define TRIANGLE 3

GAIT_DLL_API double mod1(double dividend);

GAIT_DLL_API void circle(FILE *, Vector &, double);
GAIT_DLL_API void create_leg_datafile(Leg &);
GAIT_DLL_API void create_body_datafile(Posture &);
GAIT_DLL_API void create_trajectory_plane_datafile(Robot &, double);
GAIT_DLL_API void display_robot_position(Posture &, Posture &);
GAIT_DLL_API void display_leg_position(Leg &);
GAIT_DLL_API void gait_algorithm(double time,
                                 double sampling_time,
                                 Robot &new_robot,
                                 float motion_command[3]);


#endif

// EOF
