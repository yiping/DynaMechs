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
 *     File: dmRevDCMotor.cpp
 *   Author: Scott McMillan
 *  Summary: Class definition for the only concrete example of an actuator
 *****************************************************************************/

#include "dm.h"
#include "dmRevDCMotor.hpp"

//============================================================================
// class dmRevDCMotor: public dmActuator
//============================================================================

//----------------------------------------------------------------------------
//    Summary: class constructor
// Parameters: cfg_ptr - stream reference to configuration file containing the
//             parameters needed to initialize this class's variables
//    Returns: none
//----------------------------------------------------------------------------
dmRevDCMotor::dmRevDCMotor()
      : dmActuator(),
        m_torque_constant(0.0),
        m_back_EMF_constant(0.0),
        m_armature_resistance(0.0),
        m_rotor_inertia(0.0),
        m_coulomb_friction_constant(0.0),
        m_viscous_friction_constant(0.0),
        m_max_brush_drop(0.0),
        m_half_drop_value(0.0)
{
}

//----------------------------------------------------------------------------
void dmRevDCMotor::setParameters(Float torque_constant,
                                 Float back_EMF_constant,
                                 Float armature_resistance,
                                 Float rotor_inertia,
                                 Float coulomb_friction_constant,
                                 Float viscous_friction_constant,
                                 Float max_brush_drop,
                                 Float half_drop_value)
{
    m_torque_constant = torque_constant;
    m_back_EMF_constant = back_EMF_constant;
    m_armature_resistance = armature_resistance;
    m_rotor_inertia = rotor_inertia;

    m_coulomb_friction_constant = coulomb_friction_constant;
    m_viscous_friction_constant = viscous_friction_constant;

    m_max_brush_drop = max_brush_drop;
    m_half_drop_value = half_drop_value;
}

//----------------------------------------------------------------------------
//    Summary: set the new joint velocity, and comparing with the previous
//             velocity, set the stiction flag...also set the joint velocity to
//             zero if there is a sign change from the previous value.
// Parameters: new_joint_vel - joint velocity computed by the dynamic sim.
//    Returns: new_joint_vel - same joint velocity except when there is a sign
//                             change in which case it is set to zero
//----------------------------------------------------------------------------
void dmRevDCMotor::updateStiction(Float *new_joint_vel)
{
   // Code to simulate sticking at crossover
   if (((*new_joint_vel > 0.0) && (m_prev_vel < 0.0)) ||
       ((*new_joint_vel < 0.0) && (m_prev_vel > 0.0)))
   {
      *new_joint_vel = 0.0;

      // Flag to set when joint_acc = 0 on next step regardless
      m_stiction_flag = true;
   }
}

//----------------------------------------------------------------------------
//    Summary: implement the sgn function
// Parameters: x - input number
//    Returns:  1.0 if x >  0.0
//              0.0 if x == 0.0
//             -1.0 if x <  0.0
//----------------------------------------------------------------------------
inline Float dmRevDCMotor::sgn(Float x)
{
   // My version of the sgn function.
   if (x < 0.0)
      return -1.0;
   else if (x > 0.0)
      return 1.0;
   else
      return 0.0;
}

//----------------------------------------------------------------------------
//    Summary: simulate the motor dynamics, i.e. compute the output torque
//             based on the input voltage, current velocity and load, and
//             current internal state (stiction).
// Parameters: source_voltage - voltage applied to motor (joint_input)
//             external_torque - torque load experienced by motor
//             joint_vel - current joint velocity as computed by the
//                         simulation.
//    Returns: joint torque
//----------------------------------------------------------------------------
Float dmRevDCMotor::computeTau(Float source_voltage,
                               Float external_torque,
                               Float joint_vel)
{
   Float speed_sign = sgn(joint_vel);

   if (source_voltage != 0.0)
   {
      // subtract brush drop.
      // May be made more efficient as "source_voltage" is the only
      // variable. Left as is for clarity.
      source_voltage -= sgn(source_voltage)*m_max_brush_drop *
         (1.0 - pow(0.5f, (fabs(source_voltage)/m_half_drop_value)));
   }

   // add torque developed internally by motor (Kt*i).
   Float torque = external_torque + m_torque_constant*(source_voltage -
                        joint_vel*m_back_EMF_constant)/m_armature_resistance;

   // subtract internal friction losses from torque.
   if (joint_vel == 0.0)
   {
      // coulomb friction only. (opposes torque)
      if (torque != 0.0)
      {
         if (m_coulomb_friction_constant > fabs(torque))
         {
            torque = 0.0;       // insufficient to overcome stiction.

         }
         else
         {
            torque -= sgn(torque)*m_coulomb_friction_constant;
            m_stiction_flag = false;
         }
      }
   }
   else
   {
      // friction opposes motorSpeed.
      torque -= speed_sign*m_coulomb_friction_constant +
                joint_vel*m_viscous_friction_constant;
      m_stiction_flag = false;
   }

   m_prev_vel = joint_vel;
   return torque;
}
