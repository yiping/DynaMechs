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
 *     File: dmLoadfile_dm40.cpp
 *   Author: Scott McMillan/Duane Marhefka
 *  Created: 27 July 1999
 *  Summary: load v4.0 configuration files and piece together a complete
 *           dmSystem for simulation.
 *****************************************************************************/

#include "../dm/dm.h"

#include "../dm/dmObject.hpp"
#include "../dm/dmSystem.hpp"
#include "../dm/dmArticulation.hpp"
#include "../dm/dmClosedArticulation.hpp"
#include "../dm/dmLink.hpp"
#include "../dm/dmZScrewTxLink.hpp"
#include "../dm/dmMDHLink.hpp"
#include "../dm/dmRevoluteLink.hpp"
#include "../dm/dmPrismaticLink.hpp"
#include "../dm/dmSphericalLink.hpp"
#include "../dm/dmQuaternionLink.hpp"
#include "../dm/dmMobileBaseLink.hpp"
#include "../dm/dmActuator.hpp"
#include "../dm/dmRevDCMotor.hpp"

#include "../dm/dmContactModel.hpp"
#include "../dm/dmDynamicContactModel.hpp"

#include "../dm/dmStaticRootLink.hpp"
#include "../dm/dmSecondaryJoint.hpp"
#include "../dm/dmSecondaryPrismaticJoint.hpp"
#include "../dm/dmSecondaryRevoluteJoint.hpp"
#include "../dm/dmSecondarySphericalJoint.hpp"


#include "dmLoadFile.h"
#include "glLoadModels.h"

#include <typeinfo>

#if defined(__APPLE__)
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif


int line_num40;

const int NAME_SIZE = 256;
char object_name40[NAME_SIZE];

//----------------------------------------------------------------------------
bool getStringParameter40(ifstream &cfg_ptr, const char *label, char *name)
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
void getGraphicsModel40(ifstream &cfg_ptr, dmLink *link)
{
   if (getStringParameter40(cfg_ptr, "Graphics_Model", object_name40))
   {
      GLuint *dlist = new GLuint;
      *dlist = glLoadModel(object_name40);
      link->setUserData((void *) dlist);
   }
}

//----------------------------------------------------------------------------
void setContactParameters40(dmRigidBody *body, ifstream &cfg_ptr)
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


	// DM 5.0 | for extended contact model
	readConfigParameterLabel(cfg_ptr, "Number_of_Dynamic_Contact_Points");
	cfg_ptr >> num_points;

	if (num_points > 0)
	{
	  readConfigParameterLabel(cfg_ptr, "Dynamic_Contact_Locations");

	  CartesianVector *pos = new CartesianVector[num_points];

	  for (unsigned int i=0; i<num_points; i++)
	  {
		 cfg_ptr >> pos[i][0]
		         >> pos[i][1]
		         >> pos[i][2];
	  }

	  dmDynamicContactModel *c_model = new dmDynamicContactModel();
	  c_model->setContactPoints(num_points, pos);
	  body->addForce(c_model);

	  delete [] pos;
	} 
}

//----------------------------------------------------------------------------
void setRigidBodyParameters40(dmRigidBody *body, ifstream &cfg_ptr)
{
   unsigned int i;

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

   setContactParameters40(body, cfg_ptr);
}


//----------------------------------------------------------------------------
void setJointFriction40(dmLink *link, ifstream &cfg_ptr)
{
   Float joint_friction;

   readConfigParameterLabel(cfg_ptr, "Joint_Friction");
   cfg_ptr >> joint_friction;

   link->setJointFriction(joint_friction);
}

//----------------------------------------------------------------------------
void setMDHParameters40(dmMDHLink *link, ifstream &cfg_ptr)
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
void setRevDCMotorParameters40(dmRevDCMotor *actuator, ifstream &cfg_ptr)
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
void setActuator40(dmRevoluteLink *link, ifstream &cfg_ptr)
{
   int actuator_type;

   readConfigParameterLabel(cfg_ptr, "Actuator_Type");
   cfg_ptr >> actuator_type;

   if (actuator_type == NOMOTOR)
   {
      setJointFriction40(link, cfg_ptr);
   }
   else if (actuator_type == DCMOTOR)
   {
      dmRevDCMotor *actuator = new dmRevDCMotor();
      setRevDCMotorParameters40(actuator, cfg_ptr);
      link->setActuator(actuator);
   }
   else
   {
      cerr << "Error: invalid Actuator_Type\n";
      exit(3);
   }
}

//----------------------------------------------------------------------------
void setSphericalLinkParameters40(dmSphericalLink *link, ifstream &cfg_ptr)
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

   setJointFriction40(link, cfg_ptr);
}

//----------------------------------------------------------------------------
void setQuaternionLinkParameters40(dmQuaternionLink *link, ifstream &cfg_ptr)
{
   // Joint info:
   CartesianVector p;
   readConfigParameterLabel(cfg_ptr, "Position_From_Inboard_Link");
   cfg_ptr >> p[0] >> p[1] >> p[2];
   link->setJointOffset(p);

   Float q[4];
   readConfigParameterLabel(cfg_ptr, "Orientation_Quat");
   cfg_ptr >> q[0] >> q[1] >> q[2] >> q[3];

   Float qd[3];
   readConfigParameterLabel(cfg_ptr, "Initial_Angular_Velocity");
   cfg_ptr >> qd[0] >> qd[1] >> qd[2];
   link->setState(q, qd);

   setJointFriction40(link, cfg_ptr);
}

//----------------------------------------------------------------------------
void setMobileBaseParameters40(dmMobileBaseLink *ref, ifstream &cfg_ptr)
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
void setSecondaryJoint40(ifstream &cfg_ptr, dmSecondaryJoint *joint,
                         dmClosedArticulation *articulation)
{
   register int i;

   CartesianVector a_pos;
   CartesianVector b_pos;
   RotationMatrix a_rot;
   RotationMatrix b_rot;

   joint->setArticulation(articulation);

   if (getStringParameter40(cfg_ptr, "Link_A_Name", object_name40))
   {
      dmObject *link_A = dmuFindObject( object_name40, articulation );
      if (link_A == NULL)
      {
         cerr << "Invalid secondary joint link A: '"
              << object_name40 << "'." << endl;
         exit(3);
      }

      if (  (typeid(*link_A) == typeid(dmStaticRootLink)) ||
            (typeid(*link_A) == typeid(dmMobileBaseLink)) ||
            (typeid(*link_A) == typeid(dmPrismaticLink)) ||
            (typeid(*link_A) == typeid(dmRevoluteLink)) ||
            (typeid(*link_A) == typeid(dmSphericalLink)) ||
            (typeid(*link_A) == typeid(dmQuaternionLink)) ||
            (typeid(*link_A) == typeid(dmZScrewTxLink)) )
      {
         joint->setLinkA( dynamic_cast<dmLink *>(link_A) );
      }
      else
      {
         cerr << "Invalid secondary joint link type: "
              << (typeid(*link_A)).name() << "." << endl;
         exit(3);
      }
   }
   else
   {
      cerr << "Secondary joint links must have names." << endl;
      exit(3);
   }

   if (getStringParameter40(cfg_ptr, "Link_B_Name", object_name40))
   {
      dmObject *link_B = dmuFindObject( object_name40, articulation );
      if (link_B == NULL)
      {
         cerr << "Invalid secondary joint link B: '"
              << object_name40 << "'." << endl;
         exit(3);
      }

      if (  (typeid(*link_B) == typeid(dmStaticRootLink)) ||
            (typeid(*link_B) == typeid(dmMobileBaseLink)) ||
            (typeid(*link_B) == typeid(dmPrismaticLink)) ||
            (typeid(*link_B) == typeid(dmRevoluteLink)) ||
            (typeid(*link_B) == typeid(dmSphericalLink)) ||
            (typeid(*link_B) == typeid(dmQuaternionLink)) ||
            (typeid(*link_B) == typeid(dmZScrewTxLink)) )
      {
         joint->setLinkB( dynamic_cast<dmLink *>(link_B) );
      }
      else
      {
         cerr << "Invalid secondary joint link type: "
              << (typeid(*link_B)).name() << "." << endl;
         exit(3);
      }
   }
   else
   {
      cerr << "Secondary joint links must have names." << endl;
      exit(3);
   }

   readConfigParameterLabel(cfg_ptr, "Joint_A_Position");
   cfg_ptr >> a_pos[0] >> a_pos[1] >> a_pos[2];

   readConfigParameterLabel(cfg_ptr, "Rotation_Matrix_A");
   for (i = 0; i < 3; i++)
   {
      cfg_ptr >> a_rot[i][0] >> a_rot[i][1] >> a_rot[i][2];
   }

   readConfigParameterLabel(cfg_ptr, "Joint_B_Position");
   cfg_ptr >> b_pos[0] >> b_pos[1] >> b_pos[2];

   readConfigParameterLabel(cfg_ptr, "Rotation_Matrix_B");
   for (i = 0; i < 3; i++)
   {
      cfg_ptr >> b_rot[i][0] >> b_rot[i][1] >> b_rot[i][2];
   }

   joint->setKinematics( a_pos, b_pos, a_rot, b_rot );

   Float joint_friction;
   readConfigParameterLabel(cfg_ptr, "Joint_Friction");
   cfg_ptr >> joint_friction;
   joint->setJointFriction(joint_friction);
}

//----------------------------------------------------------------------------
void setStabilization40(ifstream &cfg_ptr, dmSecondaryJoint *joint)
{
   char stabName[256];

   readConfigParameterLabel(cfg_ptr, "Stabilization");
   cfg_ptr >> stabName;

   if (strcmp(stabName, "NONE") == 0)
   {
      joint->setStabilizationType(dmSecondaryJoint::NONE);
   }
   else if (strcmp(stabName, "BAUMGARTE") == 0)
   {
      joint->setStabilizationType(dmSecondaryJoint::BAUMGARTE);
   }
   else if (strcmp(stabName, "SPRING_DAMPER") == 0)
   {
      joint->setStabilizationType(dmSecondaryJoint::SPRING_DAMPER);
   }
   else
   {
      cerr << "Unrecognizable stabilization type: " << stabName << endl;
      exit(3);
   }
}


//----------------------------------------------------------------------------
void setSecondaryPrismaticJoint40(ifstream &cfg_ptr,
                                  dmSecondaryPrismaticJoint *joint)
{
   Float linear_constraint_spring;
   Float linear_constraint_damper;
   Float angular_constraint_spring;
   Float angular_constraint_damper;

   readConfigParameterLabel(cfg_ptr, "Position_Constraint_Spring");
   cfg_ptr >> linear_constraint_spring;

   readConfigParameterLabel(cfg_ptr, "Position_Constraint_Damper");
   cfg_ptr >> linear_constraint_damper;

   readConfigParameterLabel(cfg_ptr, "Orientation_Constraint_Spring");
   cfg_ptr >> angular_constraint_spring;

   readConfigParameterLabel(cfg_ptr, "Orientation_Constraint_Damper");
   cfg_ptr >> angular_constraint_damper;

   joint->setConstraintParams(linear_constraint_spring,
                              linear_constraint_damper,
                              angular_constraint_spring,
                              angular_constraint_damper);
}


//----------------------------------------------------------------------------
void setSecondaryRevoluteJoint40(ifstream &cfg_ptr,
                                 dmSecondaryRevoluteJoint *joint)
{
   Float linear_constraint_spring;
   Float linear_constraint_damper;
   Float angular_constraint_spring;
   Float angular_constraint_damper;

   readConfigParameterLabel(cfg_ptr, "Position_Constraint_Spring");
   cfg_ptr >> linear_constraint_spring;

   readConfigParameterLabel(cfg_ptr, "Position_Constraint_Damper");
   cfg_ptr >> linear_constraint_damper;

   readConfigParameterLabel(cfg_ptr, "Orientation_Constraint_Spring");
   cfg_ptr >> angular_constraint_spring;

   readConfigParameterLabel(cfg_ptr, "Orientation_Constraint_Damper");
   cfg_ptr >> angular_constraint_damper;

   joint->setConstraintParams(linear_constraint_spring,
                              linear_constraint_damper,
                              angular_constraint_spring,
                              angular_constraint_damper);
}

//----------------------------------------------------------------------------
void setSecondarySphericalJoint40(ifstream &cfg_ptr,
                                  dmSecondarySphericalJoint *joint)
{
   Float linear_constraint_spring;
   Float linear_constraint_damper;

   readConfigParameterLabel(cfg_ptr, "Position_Constraint_Spring");
   cfg_ptr >> linear_constraint_spring;

   readConfigParameterLabel(cfg_ptr, "Position_Constraint_Damper");
   cfg_ptr >> linear_constraint_damper;


   joint->setConstraintParams(linear_constraint_spring,
                              linear_constraint_damper);
}


//----------------------------------------------------------------------------
void parseBranch40(ifstream &cfg_ptr,
                   dmArticulation *articulation,
                   dmLink *parent)
{
   char *tok;

   dmLink* last_link = parent;

   for (;;)
   {
      // get next token
      tok = getNextToken(cfg_ptr, line_num40);

      // allocate a link and pass stream reference
      if (strcmp(tok, "Branch") == 0)
      {
//           if (last_link == NULL)
//           {
//              cerr << "dmLoadfile_dm::parseBranch error: cannot branch "
//                   << "immediately inside a branch." << endl;
//              exit(4);
//           }
         parseToBlockBegin(cfg_ptr, line_num40);

         parseBranch40(cfg_ptr, articulation, last_link);
      }
      else if (strcmp(tok, "StaticRootLink") == 0)
      {
         parseToBlockBegin(cfg_ptr, line_num40);

         dmStaticRootLink *link = new dmStaticRootLink();

         if (getStringParameter40(cfg_ptr, "Name", object_name40))
         {
            link->setName(object_name40);
         }

         getGraphicsModel40(cfg_ptr, link);

         parseToBlockEnd(cfg_ptr, line_num40);

         articulation->addLink(link, last_link);
         last_link = (dmLink *) link;
      }
      else if (strcmp(tok, "MobileBaseLink") == 0)
      {
         parseToBlockBegin(cfg_ptr, line_num40);

         dmMobileBaseLink *link = new dmMobileBaseLink();

         if (getStringParameter40(cfg_ptr, "Name", object_name40))
         {
            link->setName(object_name40);
         }

         getGraphicsModel40(cfg_ptr, link);

         setRigidBodyParameters40(link, cfg_ptr);
         setMobileBaseParameters40(link, cfg_ptr);

         parseToBlockEnd(cfg_ptr, line_num40);

         articulation->addLink(link, last_link);
         last_link = (dmLink *) link;
      }
      else if (strcmp(tok, "RevoluteLink") == 0)
      {
         parseToBlockBegin(cfg_ptr, line_num40);

         dmRevoluteLink *link = new dmRevoluteLink();

         if (getStringParameter40(cfg_ptr, "Name", object_name40))
         {
            link->setName(object_name40);
         }

         getGraphicsModel40(cfg_ptr, link);

         setRigidBodyParameters40(link, cfg_ptr);
         setMDHParameters40(link, cfg_ptr);
         setActuator40(link, cfg_ptr);

         parseToBlockEnd(cfg_ptr, line_num40);

         articulation->addLink(link, last_link);
         last_link = (dmLink *) link;
      }
      else if (strcmp(tok, "PrismaticLink") == 0)
      {
         parseToBlockBegin(cfg_ptr, line_num40);

         dmPrismaticLink *link = new dmPrismaticLink();

         if (getStringParameter40(cfg_ptr, "Name", object_name40))
         {
            link->setName(object_name40);
         }

         getGraphicsModel40(cfg_ptr, link);

         setRigidBodyParameters40(link, cfg_ptr);
         setMDHParameters40(link, cfg_ptr);
         setJointFriction40(link, cfg_ptr);

         parseToBlockEnd(cfg_ptr, line_num40);

         articulation->addLink(link, last_link);
         last_link = link;
      }
      else if (strcmp(tok, "SphericalLink") == 0)
      {
         parseToBlockBegin(cfg_ptr, line_num40);

         dmSphericalLink *link = new dmSphericalLink();

         if (getStringParameter40(cfg_ptr, "Name", object_name40))
         {
            link->setName(object_name40);
         }

         getGraphicsModel40(cfg_ptr, link);

         setRigidBodyParameters40(link, cfg_ptr);
         setSphericalLinkParameters40(link, cfg_ptr);

         parseToBlockEnd(cfg_ptr, line_num40);

         articulation->addLink(link, last_link);
         last_link = link;
      }
      else if (strcmp(tok, "QuaternionLink") == 0)
      {
         parseToBlockBegin(cfg_ptr, line_num40);

         dmQuaternionLink *link = new dmQuaternionLink;

         if (getStringParameter40(cfg_ptr, "Name", object_name40))
         {
            link->setName(object_name40);
         }

         getGraphicsModel40(cfg_ptr, link);

         setRigidBodyParameters40(link, cfg_ptr);
         setQuaternionLinkParameters40(link, cfg_ptr);

         parseToBlockEnd(cfg_ptr, line_num40);

         articulation->addLink(link, last_link);
         last_link = link;
      }
      else if ((strcmp(tok, "ZScrewTxLink") == 0) ||
               (strcmp(tok, "ZScrewLinkTx") == 0))    // latter is deprecated
      {
         parseToBlockBegin(cfg_ptr, line_num40);

         bool got_name = getStringParameter40(cfg_ptr, "Name", object_name40);

         Float d, theta;

         // Get Inboard to Chain base transformation info.
         readConfigParameterLabel(cfg_ptr, "ZScrew_Parameters");
         cfg_ptr >> d >> theta;

         dmZScrewTxLink *link = new dmZScrewTxLink(d, theta);
         if (got_name)
         {
            link->setName(object_name40);
         }
         parseToBlockEnd(cfg_ptr, line_num40);

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
              << ", line " << line_num40 <<endl;
         exit(3);
      }
   }
}

//----------------------------------------------------------------------------
void parseSecondaryJoints40(ifstream &cfg_ptr,
                            dmClosedArticulation *articulation)
{
   char *tok;

   for (;;)
   {
      // get next token
      tok = getNextToken(cfg_ptr, line_num40);

      if (strcmp(tok, "HardRevoluteJoint") == 0)
      {
         parseToBlockBegin(cfg_ptr, line_num40);

         dmSecondaryRevoluteJoint *joint = new dmSecondaryRevoluteJoint;

         if (getStringParameter40(cfg_ptr, "Name", object_name40))
         {
            joint->setName(object_name40);
         }

         setStabilization40(cfg_ptr, joint);
         setSecondaryJoint40(cfg_ptr, joint, articulation);
         setSecondaryRevoluteJoint40(cfg_ptr, joint);

         parseToBlockEnd(cfg_ptr, line_num40);

         articulation->addHardSecondaryJoint(joint);
      }
      else if (strcmp(tok, "SoftRevoluteJoint") == 0)
      {
         parseToBlockBegin(cfg_ptr, line_num40);

         dmSecondaryRevoluteJoint *joint = new dmSecondaryRevoluteJoint();

         if (getStringParameter40(cfg_ptr, "Name", object_name40))
         {
            joint->setName(object_name40);
         }

         setSecondaryJoint40(cfg_ptr, joint, articulation);
         setSecondaryRevoluteJoint40(cfg_ptr, joint);

         parseToBlockEnd(cfg_ptr, line_num40);

         articulation->addSoftSecondaryJoint(joint);
      }
      else if (strcmp(tok, "HardPrismaticJoint") == 0)
      {
         parseToBlockBegin(cfg_ptr, line_num40);

         dmSecondaryPrismaticJoint *joint = new dmSecondaryPrismaticJoint();

         if (getStringParameter40(cfg_ptr, "Name", object_name40))
         {
            joint->setName(object_name40);
         }

         setStabilization40(cfg_ptr, joint);
         setSecondaryJoint40(cfg_ptr, joint, articulation);
         setSecondaryPrismaticJoint40(cfg_ptr, joint);

         parseToBlockEnd(cfg_ptr, line_num40);

         articulation->addHardSecondaryJoint(joint);
      }
      else if (strcmp(tok, "SoftPrismaticJoint") == 0)
      {
         parseToBlockBegin(cfg_ptr, line_num40);

         dmSecondaryPrismaticJoint *joint = new dmSecondaryPrismaticJoint();

         if (getStringParameter40(cfg_ptr, "Name", object_name40))
         {
            joint->setName(object_name40);
         }

         setSecondaryJoint40(cfg_ptr, joint, articulation);
         setSecondaryPrismaticJoint40(cfg_ptr, joint);

         parseToBlockEnd(cfg_ptr, line_num40);

         articulation->addSoftSecondaryJoint(joint);
      }
      else if (strcmp(tok, "HardSphericalJoint") == 0)
      {
         parseToBlockBegin(cfg_ptr, line_num40);

         dmSecondarySphericalJoint *joint = new dmSecondarySphericalJoint();

         if (getStringParameter40(cfg_ptr, "Name", object_name40))
         {
            joint->setName(object_name40);
         }

         setStabilization40(cfg_ptr, joint);
         setSecondaryJoint40(cfg_ptr, joint, articulation);
         setSecondarySphericalJoint40(cfg_ptr, joint);

         parseToBlockEnd(cfg_ptr, line_num40);

         articulation->addHardSecondaryJoint(joint);
      }
      else if (strcmp(tok, "SoftSphericalJoint") == 0)
      {
         parseToBlockBegin(cfg_ptr, line_num40);

         dmSecondarySphericalJoint *joint = new dmSecondarySphericalJoint();

         if (getStringParameter40(cfg_ptr, "Name", object_name40))
         {
            joint->setName(object_name40);
         }

         setSecondaryJoint40(cfg_ptr, joint, articulation);
         setSecondarySphericalJoint40(cfg_ptr, joint);

         parseToBlockEnd(cfg_ptr, line_num40);

         articulation->addSoftSecondaryJoint(joint);
      }
      else if (tok[0] == BLOCK_END_CHAR)
      {
         break;
      }
      else
      {
         cerr << "Error: Invalid Secondary_Joint_Type: " << tok
              << ", line " << line_num40 << endl;
         exit(3);
      }
   }
}

//----------------------------------------------------------------------------
dmArticulation *loadArticulation40(ifstream &cfg_ptr)
{
   dmArticulation *robot = new dmArticulation();;
   if (robot == NULL)
   {
      cerr << "dmLoadFile_dm40::loadArticulation40 error: "
           << "unable to allocate dmArticulation" << endl;
      cfg_ptr.close();
      exit(4);
   }

   parseToBlockBegin(cfg_ptr, line_num40);

   // label the system object
   char sys_name[NAME_SIZE];

   if (getStringParameter40(cfg_ptr, "Name", sys_name))
   {
      robot->setName(sys_name);
   }

   char object_name40[256];
   if (getStringParameter40(cfg_ptr, "Graphics_Model", object_name40) &&
       (object_name40[0] != '\0'))
   {
      GLuint *dlist = new GLuint;
      *dlist = glLoadModel(object_name40);
      robot->setUserData((void *) dlist);
   }

   // ================== NEW Get Reference System info. =====================
   CartesianVector pos;
   dmQuaternion quat;

   readConfigParameterLabel(cfg_ptr, "Position");
   cfg_ptr >> pos[0] >> pos[1] >> pos[2];
   readConfigParameterLabel(cfg_ptr, "Orientation_Quat");
   cfg_ptr >> quat[0] >> quat[1] >> quat[2] >> quat[3];

   robot->setRefSystem(quat, pos);
   // ========================================================================

   // there have to be links
   parseBranch40(cfg_ptr, robot, NULL);

   return robot;
}

//----------------------------------------------------------------------------
dmClosedArticulation *loadClosedArticulation40(ifstream &cfg_ptr)
{
   dmClosedArticulation *robot = new dmClosedArticulation();;
   if (robot == NULL)
   {
      cerr << "dmLoadFile_dm40::loadClosedArticulation40 error: "
           << "unable to allocate dmClosedArticulation" << endl;
      cfg_ptr.close();
      exit(4);
   }

   parseToBlockBegin(cfg_ptr, line_num40);

   // label the system object
   char sys_name[NAME_SIZE];

   if (getStringParameter40(cfg_ptr, "Name", sys_name))
   {
      robot->setName(sys_name);
   }

   char object_name40[256];
   if (getStringParameter40(cfg_ptr, "Graphics_Model", object_name40) &&
       (object_name40[0] != '\0'))
   {
      GLuint *dlist = new GLuint;
      *dlist = glLoadModel(object_name40);
      robot->setUserData((void *) dlist);
   }

   // ================== NEW Get Reference System info. =====================
   CartesianVector pos;
   dmQuaternion quat;

   readConfigParameterLabel(cfg_ptr, "Position");
   cfg_ptr >> pos[0] >> pos[1] >> pos[2];
   readConfigParameterLabel(cfg_ptr, "Orientation_Quat");
   cfg_ptr >> quat[0] >> quat[1] >> quat[2] >> quat[3];

   robot->setRefSystem(quat, pos);

   // ======== Load in the tree structure of the closed articulation.========
   readConfigParameterLabel(cfg_ptr, "TreeStructure");
   parseToBlockBegin(cfg_ptr, line_num40);
   parseBranch40(cfg_ptr, robot, NULL);

   // ======= Load in the secondary joints of the closed articulation. ======
   readConfigParameterLabel(cfg_ptr, "SecondaryJoints");
   parseToBlockBegin(cfg_ptr, line_num40);
   parseSecondaryJoints40(cfg_ptr, robot);
   parseToBlockEnd(cfg_ptr, line_num40);

   // ======= Initialize internal data structures for kinematic loops. ======
   robot->initKinematicLoopVars();

   return robot;
}

//----------------------------------------------------------------------------
dmSystem *dmLoadFile_dm40(ifstream &cfg_ptr)
{
   line_num40 = 1;
   bool system_flag = true;

   dmSystem *robot = NULL;

   do
   {
      char *tok = getNextToken(cfg_ptr, line_num40);

      if (strcmp(tok, "Articulation") == 0)
      {
         robot = loadArticulation40(cfg_ptr);
         system_flag = false;
      }
//    else if (strcmp(tok, "SerialChain") == 0)
//    {
//       robot = loadSerialChain40(cfg_ptr);
//       system_flag = false;
//    }
      else if (strcmp(tok, "ClosedArticulation") == 0)
      {
         robot = loadClosedArticulation40(cfg_ptr);
         system_flag = false;
      }
      else
      {
         cerr << "dmLoadFile_dm40 error: unknown token on line " << line_num40
              << ": " << tok << endl;
      }
   } while (system_flag);

   return robot;
}
