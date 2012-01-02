/*****************************************************************************
 * DynaMechs: A Multibody Dynamic Simulation Library
 *
 * Copyright (C) 1994-2001  Scott McMillan   All Rights Reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the Free
 * Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *****************************************************************************
 *     File: dmRevDCMotor.hpp
 *   Author: Scott McMillan
 *  Summary: Class definition for the only concrete example of an actuator
 *****************************************************************************/

#ifndef _DM_REV_DC_MOTOR_HPP
#define _DM_REV_DC_MOTOR_HPP

#include "dm.h"
#include "dmActuator.hpp"

//======================================================================
/**

This currently the only concrete example of an actuator.  It models a DC motor
that drives a revolute joint.  The loader function expects the configuration
file to containing the following lines after the {\tt Actuator\_Type} line:
\begin{verbatim}
    Motor_Torque_Constant              K_t
    Motor_BackEMF_Constant             K_b
    Motor_Armature_Resistance          R_a
    Motor_Inertia                      J_m
    Motor_Coulomb_Friction_Constant    u_c
    Motor_Viscous_Friction_Constant    u_v
    Motor_Max_Brush_Drop               V_max
    Motor_Half_Drop_Value              V_half
\end{verbatim}
where all of these parameters have been adjusted (where necessary) for any
gearing and are the values seen from the outside.  Note that the {\tt
setParameters} function has been provided to set these parameters under
programmer control.  {\em I either need to add a getParameters function or I
need to implement individual get/set functions for each parameter.}

Note that with significant gearing a small rotor inertia ({\tt Motor\_Inertia})
can easily become a significant portion of a link's inertia.  This value is
automatically added to the link's spatial inertia matrix when this actuator is
{\em attached} to the {\tt dmRevoluteLink} object, and the {\tt
getRotorInertia} function provides the link access to this value.

The {\tt computeTau} function simulates the motor dynamics.  I recommend that
you decipher the source code if you want to learn more, or consult Wrus
Kristiansen Master's Thesis from the Naval Postgraduate School.

See also {\tt dmActuator}.

 */

class DM_DLL_API dmRevDCMotor : public dmActuator
{
public:
   ///
   dmRevDCMotor();
   ///
   virtual ~dmRevDCMotor() {};

   ///
   void setParameters(Float torque_constant,
                      Float back_EMF_constant,
                      Float armature_resistance,
                      Float rotor_inertia,
                      Float coulomb_friction_constant,
                      Float viscous_friction_constant,
                      Float max_brush_drop,
                      Float half_drop_value);

   ///
   inline Float getRotorInertia() const { return m_rotor_inertia; }

   ///
   void updateStiction(Float *new_joint_vel);  /* FIXME - use reference? */
   ///
   Float computeTau(Float sourceVoltage,
                    Float extTau,
                    Float joint_vel);

private:
   // not implemented
   dmRevDCMotor(const dmRevDCMotor &);
   dmRevDCMotor &operator=(const dmRevDCMotor &);

   Float sgn(Float x);

private:
   Float m_torque_constant;
   Float m_back_EMF_constant;
   Float m_armature_resistance;
   Float m_rotor_inertia;
   Float m_coulomb_friction_constant;
   Float m_viscous_friction_constant;
   Float m_max_brush_drop;
   Float m_half_drop_value;
};

#endif
