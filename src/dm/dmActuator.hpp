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
 *     File: dmActuator.hpp
 *   Author: Scott McMillan
 *  Summary: Class definition for actuator abstract base class
 *****************************************************************************/

#ifndef _DM_ACTUATOR_HPP
#define _DM_ACTUATOR_HPP

#include "dm.h"
#include "dmObject.hpp"

//======================================================================
/**

This is the abstract base class of an experimental (underdeveloped) class
hierarchy of actuators.  This and the (currently) only derived class ({\tt
dmRevDCMotor}) give an example of one way in which this could be done.  As it
is currently implemented, an actuator is a member of the {\tt dmMDHLink} class
and is added to this class in much the same way that a contact model is added
to a rigid body.

Part of the motor model provided with this class is stiction.  The presence of
stiction requires that the velocity must go to zero when ever a sign change in
velocity of the simulated joint is detected.  This is important because it
allows the model to enter the static friction domain.  In order to accomplish
this though, the joint velocity must be overridden whenever a sign change
occurs.  First, the stiction simulation must be initialized by calling the {\tt
initStiction} function with the initial joint velocity.  Then the {\tt
updateStiction} function, which is implemented by the derived actuator classes,
is called at the beginning of the link's {\tt ABForwardKinematics} function.
Finally, the link's {\tt ABForwardAcceleration} function must call the {\tt
getStictionFlag} function to determine if the joint acceleration should be
overridden to zero as well.  There is no gaurantee that this model is correct,
but it looks pretty good when it works.

Note that when an actuator is present in a link class the meaning of the
parameter to the {\tt setJointInput} function changes from a joint torque/force
to the actuator's input (a voltage in the case of {\tt dmRevDCMotor}).  The
{\tt computeTau} function is called by the link's {\tt ABBackwardDynamics}
function and passes the link's joint input value in the first parameter, any
external torques about the motor/joint axis that are present in the second
parameter, and the current joint velocity in the third parameter.  This
function is a pure virtual function, which is implemented by the derived
classes to simulate the motor dynamics.

Also note that all of these functions were developed assuming that Euler
integration would be used.  I do not know what affect a multistep method like
Runge-Kutte would have on the stiction flag updating methods.

 */
class DM_DLL_API dmActuator : public dmObject
{
public:
   ///
   dmActuator();
   ///
   virtual ~dmActuator() {};

   ///
   void initStiction(Float qd);
   ///
   inline bool getStictionFlag() const { return  m_stiction_flag; }

   ///
   virtual void updateStiction(Float *new_joint_vel) = 0;
   ///
   virtual Float computeTau(Float source_voltage,
                            Float external_tau,
                            Float joint_vel) = 0;

protected:
   // not implemented
   dmActuator(const dmActuator &);
   dmActuator &operator=(const dmActuator &);

protected:
   bool  m_stiction_flag; // Used to indicate stiction (zero vel. crossover).
   Float m_prev_vel;
};

#endif
