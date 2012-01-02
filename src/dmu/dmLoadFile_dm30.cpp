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
 *     File: dmLoadfile_dm30.cpp
 *   Author: Scott McMillan
 *  Created: 27 July 1999
 *  Summary: load v3.0 configuration files and piece together a complete
 *           dmSystem for simulation.
 *****************************************************************************/

#include "../dm/dm.h"

#ifdef WIN32
#include <windows.h>
#endif

#include <GL/gl.h>

#include "../dm/dmObject.hpp"
#include "../dm/dmSystem.hpp"
#include "../dm/dmArticulation.hpp"
#include "../dm/dmLink.hpp"
#include "../dm/dmZScrewTxLink.hpp"
#include "../dm/dmMDHLink.hpp"
#include "../dm/dmRevoluteLink.hpp"
#include "../dm/dmPrismaticLink.hpp"
#include "../dm/dmSphericalLink.hpp"
#include "../dm/dmMobileBaseLink.hpp"
#include "../dm/dmActuator.hpp"
#include "../dm/dmRevDCMotor.hpp"

#include "../dm/dmContactModel.hpp"

#include "dmLoadFile.h"
#include "glLoadModels.h"

int line_num30;

const int NAME_SIZE = 256;
char object_name30[NAME_SIZE];

//----------------------------------------------------------------------------
bool getStringParameter30(ifstream &cfg_ptr, const char *label, char *name)
{
   readConfigParameterLabel(cfg_ptr, label);

   if (cfg_ptr.getline(name, NAME_SIZE, '\042'))
   {
      if (cfg_ptr.getline(name, NAME_SIZE, '\042'))
      {
         if (strlen(name) > 0)
         {
            //cerr << "Got name: " << name << endl;
            return true;
         }
      }
   }
   return false;
}

//----------------------------------------------------------------------------
void getGraphicsModel30(ifstream &cfg_ptr, dmLink *link)
{
   if (getStringParameter30(cfg_ptr, "Graphics_Model", object_name30))
   {
      GLuint *dlist = new GLuint;
      *dlist = glLoadModel(object_name30);
      link->setUserData((void *) dlist);
   }
}

//----------------------------------------------------------------------------
void setContactParameters30(dmRigidBody *body, ifstream &cfg_ptr)
{
   unsigned int num_points;
   readConfigParameterLabel(cfg_ptr, "Number_of_Contact_Points");
   cfg_ptr >> num_points;

   if (num_points > 0)
   {
      readConfigParameterLabel(cfg_ptr, "Contact_Locations");

      CartesianVector *pos = new CartesianVector[num_points];

      for (unsigned int i=0; i<num_points; i++)
      {
         cfg_ptr >> pos[i][0]
                 >> pos[i][1]
                 >> pos[i][2];
      }

      dmContactModel *contact_model = new dmContactModel();
      contact_model->setContactPoints(num_points, pos);
      //contact_model->setEnvironment(&G_environment);
      body->addForce(contact_model);

      delete [] pos;
   }
}

//----------------------------------------------------------------------------
void setRigidBodyParameters30(dmRigidBody *body, ifstream &cfg_ptr)
{
   register int i;

// dynamic properties:
   Float mass;
   CartesianTensor I_bar;
   CartesianVector cg_pos;

   readConfigParameterLabel(cfg_ptr, "Mass");
   cfg_ptr >> mass;
   readConfigParameterLabel(cfg_ptr, "Inertia");
   for (i = 0; i < 3; i++)
   {
      cfg_ptr >> I_bar[i][0] >> I_bar[i][1] >> I_bar[i][2];
   }
   readConfigParameterLabel(cfg_ptr, "Center_of_Gravity");
   cfg_ptr >> cg_pos[0] >> cg_pos[1] >> cg_pos[2];

   body->setInertiaParameters(mass, I_bar, cg_pos);

#ifdef DM_HYDRODYNAMICS
// hydrodynamic properties:
   Float volume;
   SpatialTensor I_added_mass;
   CartesianVector cb_pos;
   int drag_axis;
   Float cyl_min;
   Float cyl_max;
   Float cyl_radius;
   Float C_d;

   readConfigParameterLabel(cfg_ptr, "Volume");
   cfg_ptr >> volume;

   readConfigParameterLabel(cfg_ptr, "Added_Mass");
   for (i = 0; i < 6; i++)
      for (unsigned int j = 0; j < 6; j++)
         cfg_ptr >> I_added_mass[i][j];

   readConfigParameterLabel(cfg_ptr, "Center_of_Buoyancy");
   cfg_ptr >> cb_pos[0] >> cb_pos[1] >> cb_pos[2];

// drag parameters
   readConfigParameterLabel(cfg_ptr, "Drag_Axis");
   cfg_ptr >> drag_axis;

   readConfigParameterLabel(cfg_ptr, "Cylinder_Bounds");
   cfg_ptr >> cyl_min >> cyl_max;

   readConfigParameterLabel(cfg_ptr, "Cylinder_Radius");
   cfg_ptr >> cyl_radius;

   readConfigParameterLabel(cfg_ptr, "Drag_Coefficient");
   cfg_ptr >> C_d;

   body->setHydrodynamicParameters(volume, I_added_mass, cb_pos,
                                   drag_axis,
                                   cyl_min, cyl_max, cyl_radius, C_d);
#endif

   setContactParameters30(body, cfg_ptr);
}


//----------------------------------------------------------------------------
void setJointFriction30(dmLink *link, ifstream &cfg_ptr)
{
   Float joint_friction;

   readConfigParameterLabel(cfg_ptr, "Joint_Friction");
   cfg_ptr >> joint_friction;

   link->setJointFriction(joint_friction);
}

//----------------------------------------------------------------------------
void setMDHParameters30(dmMDHLink *link, ifstream &cfg_ptr)
{
   Float a, alpha, d, theta;
   Float q, qd;

   readConfigParameterLabel(cfg_ptr, "MDH_Parameters");
   cfg_ptr >> a >> alpha >> d >> theta;

   link->setMDHParameters(a, alpha, d, theta);
   link->getState(&q, &qd);  // either d or theta could be q this retrieves it

   readConfigParameterLabel(cfg_ptr, "Initial_Joint_Velocity");
   cfg_ptr >> qd;

   link->setState(&q, &qd);  // essentially sets the joint velocity

   Float min, max, spring, damper;

   readConfigParameterLabel(cfg_ptr, "Joint_Limits");
   cfg_ptr >> min >> max;
   readConfigParameterLabel(cfg_ptr, "Joint_Limit_Spring_Constant");
   cfg_ptr >> spring;
   readConfigParameterLabel(cfg_ptr, "Joint_Limit_Damper_Constant");
   cfg_ptr >> damper;

   link->setJointLimits(min, max, spring, damper);
}

//----------------------------------------------------------------------------
void setRevDCMotorParameters30(dmRevDCMotor *actuator, ifstream &cfg_ptr)
{
   Float torque_constant;
   Float back_EMF_constant;
   Float armature_resistance;
   Float rotor_inertia;
   Float coulomb_friction_constant;
   Float viscous_friction_constant;
   Float max_brush_drop;
   Float half_drop_value;

   readConfigParameterLabel(cfg_ptr, "Motor_Torque_Constant");
   cfg_ptr >> torque_constant;
   readConfigParameterLabel(cfg_ptr, "Motor_BackEMF_Constant");
   cfg_ptr >> back_EMF_constant;
   readConfigParameterLabel(cfg_ptr, "Motor_Armature_Resistance");
   cfg_ptr >> armature_resistance;
   readConfigParameterLabel(cfg_ptr, "Motor_Inertia");
   cfg_ptr >> rotor_inertia;

   readConfigParameterLabel(cfg_ptr, "Motor_Coulomb_Friction_Constant");
   cfg_ptr >> coulomb_friction_constant;
   readConfigParameterLabel(cfg_ptr, "Motor_Viscous_Friction_Constant");
   cfg_ptr >> viscous_friction_constant;

   readConfigParameterLabel(cfg_ptr, "Motor_Max_Brush_Drop");
   cfg_ptr >> max_brush_drop;
   readConfigParameterLabel(cfg_ptr, "Motor_Half_Drop_Value");
   cfg_ptr >> half_drop_value;

   actuator->setParameters(torque_constant,
                           back_EMF_constant,
                           armature_resistance,
                           rotor_inertia,
                           coulomb_friction_constant,
                           viscous_friction_constant,
                           max_brush_drop,
                           half_drop_value);
}

//----------------------------------------------------------------------------
void setActuator30(dmRevoluteLink *link, ifstream &cfg_ptr)
{
   int actuator_type;

   readConfigParameterLabel(cfg_ptr, "Actuator_Type");
   cfg_ptr >> actuator_type;

   if (actuator_type == NOMOTOR)
   {
      setJointFriction30(link, cfg_ptr);
   }
   else if (actuator_type == DCMOTOR)
   {
      dmRevDCMotor *actuator = new dmRevDCMotor();
      setRevDCMotorParameters30(actuator, cfg_ptr);
      link->setActuator(actuator);
   }
   else
   {
      cerr << "Error: invalid Actuator_Type\n";
      exit(3);
   }
}

//----------------------------------------------------------------------------
//    Summary: class constructor
// Parameters: cfg_ptr - pointer to file containing required parameters
//    Returns: none
//----------------------------------------------------------------------------
void setSphericalLinkParameters30(dmSphericalLink *link, ifstream &cfg_ptr)
{
   // Joint info:
   CartesianVector p;
   readConfigParameterLabel(cfg_ptr, "Position_From_Inboard_Link");
   cfg_ptr >> p[0] >> p[1] >> p[2];
   link->setJointOffset(p);

   EulerAngles ang;
   readConfigParameterLabel(cfg_ptr, "Initial_Joint_Angles");
   cfg_ptr >> ang[0] >> ang[1] >> ang[2];

   Float qd[dmSphericalLink::NUM_DOFS];
   readConfigParameterLabel(cfg_ptr, "Initial_Angular_Velocity");
   cfg_ptr >> qd[0] >> qd[1] >> qd[2];
   link->setState(ang, qd);

   Float joint_limit[dmSphericalLink::NUM_DOFS];
   Float spring, damper;
   readConfigParameterLabel(cfg_ptr, "Axes_Limits");
   cfg_ptr >> joint_limit[0] >> joint_limit[1] >> joint_limit[2];
   readConfigParameterLabel(cfg_ptr, "Joint_Limit_Spring_Constant");
   cfg_ptr >> spring;
   readConfigParameterLabel(cfg_ptr, "Joint_Limit_Damper_Constant");
   cfg_ptr >> damper;
   link->setJointLimits(joint_limit, spring, damper);

   setJointFriction30(link, cfg_ptr);
}

//----------------------------------------------------------------------------
void setMobileBaseParameters30(dmMobileBaseLink *ref, ifstream &cfg_ptr)
{
   Float q[7];
   SpatialVector vel;

   readConfigParameterLabel(cfg_ptr, "Position");
   cfg_ptr >> q[4] >> q[5] >> q[6];
   readConfigParameterLabel(cfg_ptr, "Orientation_Quat");
   cfg_ptr >> q[0] >> q[1] >> q[2] >> q[3];

   readConfigParameterLabel(cfg_ptr, "Velocity");
   for (int i = 0; i < 6; i++)
   {
      cfg_ptr >> vel[i];
   }

   ref->setState(q, vel);
}

//----------------------------------------------------------------------------
void parseBranch30(ifstream &cfg_ptr,
                   dmArticulation *articulation,
                   dmLink *parent)
{
   char *tok;

   dmLink* last_link = parent;

   for (;;)
   {
      // get next token
      tok = getNextToken(cfg_ptr, line_num30);

      // allocate a link and pass stream reference
      if (strcmp(tok, "Branch") == 0)
      {
//           if (last_link == NULL)
//           {
//              cerr << "dmLoadfile_dm::parseBranch error: cannot branch "
//                   << "immediately inside a branch." << endl;
//              exit(4);
//           }
         parseToBlockBegin(cfg_ptr, line_num30);

         parseBranch30(cfg_ptr, articulation, last_link);
      }
      else if (strcmp(tok, "MobileBaseLink") == 0)
      {
         if (last_link != NULL)
         {
            cerr << "dmLoadfile_dm30::parseBranch30 warning: mobile base "
                 << "link should only be used for first link." << endl;
         }

         parseToBlockBegin(cfg_ptr, line_num30);

         dmMobileBaseLink *link = new dmMobileBaseLink();

         if (getStringParameter30(cfg_ptr, "Name", object_name30))
         {
            link->setName(object_name30);
         }

         getGraphicsModel30(cfg_ptr, link);

         setRigidBodyParameters30(link, cfg_ptr);
         setMobileBaseParameters30(link, cfg_ptr);

         parseToBlockEnd(cfg_ptr, line_num30);

         articulation->addLink(link, last_link);
         last_link = (dmLink *) link;
      }
      else if (strcmp(tok, "RevoluteLink") == 0)
      {
         parseToBlockBegin(cfg_ptr, line_num30);

         dmRevoluteLink *link = new dmRevoluteLink();

         if (getStringParameter30(cfg_ptr, "Name", object_name30))
         {
            link->setName(object_name30);
         }

         getGraphicsModel30(cfg_ptr, link);

         setRigidBodyParameters30(link, cfg_ptr);
         setMDHParameters30(link, cfg_ptr);
         setActuator30(link, cfg_ptr);

         parseToBlockEnd(cfg_ptr, line_num30);

         articulation->addLink(link, last_link);
         last_link = (dmLink *) link;
      }
      else if (strcmp(tok, "PrismaticLink") == 0)
      {
         parseToBlockBegin(cfg_ptr, line_num30);

         dmPrismaticLink *link = new dmPrismaticLink();

         if (getStringParameter30(cfg_ptr, "Name", object_name30))
         {
            link->setName(object_name30);
         }

         getGraphicsModel30(cfg_ptr, link);

         setRigidBodyParameters30(link, cfg_ptr);
         setMDHParameters30(link, cfg_ptr);
         setJointFriction30(link, cfg_ptr);

         parseToBlockEnd(cfg_ptr, line_num30);

         articulation->addLink(link, last_link);
         last_link = link;
      }
      else if (strcmp(tok, "SphericalLink") == 0)
      {
         parseToBlockBegin(cfg_ptr, line_num30);

         dmSphericalLink *link = new dmSphericalLink();

         if (getStringParameter30(cfg_ptr, "Name", object_name30))
         {
            link->setName(object_name30);
         }

         getGraphicsModel30(cfg_ptr, link);

         setRigidBodyParameters30(link, cfg_ptr);
         setSphericalLinkParameters30(link, cfg_ptr);

         parseToBlockEnd(cfg_ptr, line_num30);

         articulation->addLink(link, last_link);
         last_link = link;
      }
      else if ((strcmp(tok, "ZScrewTxLink") == 0) ||
               (strcmp(tok, "ZScrewLinkTx") == 0))    // latter is deprecated
      {
         parseToBlockBegin(cfg_ptr, line_num30);

         bool got_name = getStringParameter30(cfg_ptr, "Name", object_name30);

         Float d, theta;

         // Get Inboard to Chain base transformation info.
         readConfigParameterLabel(cfg_ptr, "ZScrew_Parameters");
         cfg_ptr >> d >> theta;

         dmZScrewTxLink *link = new dmZScrewTxLink(d, theta);
         if (got_name)
         {
            link->setName(object_name30);
         }
         parseToBlockEnd(cfg_ptr, line_num30);

         articulation->addLink(link, last_link);
         last_link = link;
      }
      else if (tok[0] == BLOCK_END_CHAR)
      {
         break;
      }
      else
      {
         cerr << "Error: Invalid Joint_Type: " << tok
              << ", line " << line_num30 <<endl;
         exit(3);
      }
   }
}

//----------------------------------------------------------------------------
dmArticulation *loadArticulation30(ifstream &cfg_ptr)
{
   dmArticulation *robot = new dmArticulation();;
   if (robot == NULL)
   {
      cerr << "dmLoadFile_dm30::loadArticulation30 error: "
           << "unable to allocate dmArticulation" << endl;
      cfg_ptr.close();
      exit(4);
   }

   parseToBlockBegin(cfg_ptr, line_num30);

   // label the system object
   char sys_name[NAME_SIZE];

   if (getStringParameter30(cfg_ptr, "Name", sys_name))
   {
      robot->setName(sys_name);
   }

   char object_name30[256];
   if (getStringParameter30(cfg_ptr, "Graphics_Model", object_name30) &&
       (object_name30[0] != '\0'))
   {
      cerr << "in here " << object_name30 << endl;
      GLuint *dlist = new GLuint;
      cerr << "in here" << endl;
      *dlist = glLoadModel(object_name30);
      cerr << "in here" << endl;
      robot->setUserData((void *) dlist);
   }

   // ================== NEW Get Reference System info. =====================
   CartesianVector pos;
   Quaternion quat;

   readConfigParameterLabel(cfg_ptr, "Position");
   cfg_ptr >> pos[0] >> pos[1] >> pos[2];
   readConfigParameterLabel(cfg_ptr, "Orientation_Quat");
   cfg_ptr >> quat[0] >> quat[1] >> quat[2] >> quat[3];

   robot->setRefSystem(quat, pos);
   //========================================================================

   // there have to be links
   parseBranch30(cfg_ptr, robot, NULL);

   return robot;
}

//----------------------------------------------------------------------------
dmSystem *dmLoadFile_dm30(ifstream &cfg_ptr)
{
   line_num30 = 1;
   bool system_flag = true;

   dmSystem *robot = NULL;

   do
   {
      char *tok = getNextToken(cfg_ptr, line_num30);

      if (strcmp(tok, "Articulation") == 0)
      {
         robot = loadArticulation30(cfg_ptr);
         system_flag = false;
      }
//    else if (strcmp(tok, "SerialChain") == 0)
//    {
//       robot = loadSerialChain30(cfg_ptr);
//       system_flag = false;
//    }
//    else if (strcmp(tok, "ClosedArticulation") == 0)
//    {
//       robot = loadClosedArticulation30(cfg_ptr);
//       system_flag = false;
//    }
      else
      {
         cerr << "dmLoadFile_dm30 error: unknown token on line " << line_num30
              << ": " << tok << endl;
      }
   } while (system_flag);

   return robot;
}
