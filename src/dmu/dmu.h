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
 *     File: dmu.h
 *   Author: Scott McMillan
 *  Created:
 *  Summary: A collection of miscellaneous useful functions
 *****************************************************************************/

#ifndef _DMU_H
#define _DMU_H

#if defined(WIN32) && defined(_DLL)
// The next define will come from the makefile for archive objects.
#ifdef dmu_DLL_FILE
#define DMU_DLL_API __declspec(dllexport)
#else
#define DMU_DLL_API __declspec(dllimport)
#endif
#else
#define DMU_DLL_API
#endif

#include "../dm/dm.h"

class dmObject;
class dmSystem;
class dmArticulation;
class dmEnvironment;

/**

Load a configuration file.
A configuration file reader, {\tt dmLoadFile\_dm} is being supplied with the
dmutils library that can be used to instantiate and intialize all types of
objects.

Comments begin with the hash (\#) character and continues to the end of the
line.  Blocks are delimited by braces (\{\}) and are preceded by a label
denoting the type of block.

The configuration files begin with a comment line specifying the format of its
contents.  Currently three versions are supported (two for backward
compatibility).  The three valid forms of this comment line are as follows:
\begin{verbatim}
# DynaMechs V 2.0.3 ascii
# DynaMechs V 2.1 ascii
# DynaMechs V 3.0 ascii
\end{verbatim}
The last one corresponds to the most recent version of the library and allows
the most flexible specifications of multi-body systems.  The other two are
provided for backward compatibility with previous version of this library.


{\Large \bf Format for Versions 2.0.3 and 2.1}

These configuration formats begin with a specification of 3D models to
represent various objects indexed in later blocks:
\begin{verbatim}
Graphics_Models {
        Number_Graphics_Models  2
        "models/obj_cube_center.xan"
        "models/obj_cylinder_x.xan"
}
\end{verbatim}

The top-level block is the System block which contains one reference member
block and any number of articulation blocks:
\begin{verbatim}
System {
    Name                             "some_label"    # version 2.1 format only
    (reference member info)

    (articulation 0 info)
         :
         :
    (articulation n info)
}
\end{verbatim}

Reference member blocks can be one of two types:

\begin{verbatim}
StaticRefMember {
    Name                             "some_label"    # version 2.1 format only
    Graphics_Model_Index             i

    Position                         x   y   z
    Orientation_Quat                 a_x a_y a_z  s
}
\end{verbatim}

\begin{verbatim}
DynamicRefMember {
    Name                             "some_label"    # version 2.1 format only
    Graphics_Model_Index             i

    (rigid body parameters)

    Position                         x   y   z
    Orientation_Quat                 a_x a_y a_z  s
    Velocity                         w_x w_y w_z  v_x v_y v_z
}
\end{verbatim}
where specifications for rigid body parameters can be found below and the {\tt
Velocity} parameters specify the initial spatial (angular and linear) velocity
with respect to the inertial coordinate system.

Articulation blocks are indicated by the following:
\begin{verbatim}
Articulation {
    Name                             "some_label"    # version 2.1 format only
    (links and branches)
}
\end{verbatim}

A simple serial chain of links is organized as follows:
\begin{verbatim}
Articulation {
    Name                             "some_label"    # version 2.1 format only
    (link 0)
    (link 1)
       :
       :
    (link n)
}
\end{verbatim}

A branch after link 1 is organized as follows
\begin{verbatim}
Articulation {
    Name                             "some_label"    # version 2.1 format only
    (link 0)
    (link 1)
    Branch {
        (link 2)
        (link 3)
    }
    (link 4)
       :
       :
    (links n)
}
\end{verbatim}
which has the following topology (note the first link of any branch is
connected to the reference member, r):
\begin{verbatim}
 r - 0 - 1 - 2 - 3
           \
             4 - ... - n
\end{verbatim}
Also note that branch can occur anywhere after the first link (a branch before
the first link is the same as a different articulation.  Branches can also be
nested.  This structure supports all tree topologies.

See the section on link types for specification possible ones

{\Large \bf Format for version 3.0}

This format differs from the previous versions in a number of significant ways:
\begin{itemize}
\item The Articulation class has been merged with the System class
(i.e., a system consists of only one articulation)
\item Reference members are no longer used in 3.0.  A static reference member
specification (basically a constant position and orientation) has been moved to
the specification of the System/Articulation.  A dynamic reference member is
now specified as a MobileBaseLink as the first link in the System.
\item The initial list of graphics models has been eliminated and instead of
Graphics\_Model\_Indices within the specifications of the links, the filename
is provided in the Graphics\_Model field.
\end{itemize}

The top-level block is the System block which contains all of the links (rigid
bodies and transforms) for the multibody system it contains.  The label for
this block depends on the type of system it describes.  Currently only the
Articulation type (describing a tree structure) is supported and is specified
as follows:
\begin{verbatim}
Articulation {
    Name                             "some_label"
    Graphics_Model                   "some_model_file"
    Position                         x   y   z
    Orientation_Quat                 a_x a_y a_z  s

    (link and branches)
}
\end{verbatim}
The Name is a label given to the dmArticulation object.  A graphics model can
be associated with the base by specifying a valid file in the Graphics\_Model
field.  An empty field ("") corresponds to no model.  The Position and
Orientation\_Quat specifies the position and orientation of the inertial
coordinate system relative to which the system is simulated.


The Articulation block contains any combination of branches an links to
describe any tree structure.  Some examples of these are given as follows.

A simple serial chain of links is organized as follows:
\begin{verbatim}
Articulation {
    Name                             "some_label"
    Graphics_Model                   "some_model_file"
    Position                         x   y   z
    Orientation_Quat                 a_x a_y a_z  s

    (link 0)
    (link 1)
       :
       :
    (link n)
}
\end{verbatim}

A Branch can occur anywhere (even before any links).  Branches
that occur at the beginning can be used to describe multiple systems:
\begin{verbatim}
Articulation {
    Name                             "some_label"
    Graphics_Model                   "some_model_file"
    Position                         x   y   z
    Orientation_Quat                 a_x a_y a_z  s

    Branch {
        (link 0)
        (link 1)
    }
    Branch {
        (link 2)
        (link 3)
    }
    (link 4)
       :
       :
    (links n)
}
\end{verbatim}
which has the following topology (note the first link of any branch is
connected to the System's inertial coordinate system, r) consisting of three
distinct serial chains:
\begin{verbatim}
  r - 0 - 1
  r - 2 - 3
  r - 4 - ... - n
\end{verbatim}

A Branch after a link is used to connect two or more links to the same one:
\begin{verbatim}
Articulation {
    Name                             "some_label"
    Graphics_Model                   "some_model_file"
    Position                         x   y   z
    Orientation_Quat                 a_x a_y a_z  s

    (link 0)
    (link 1)
    Branch {
        (link 2)
        (link 3)
    }
    (link 4)
       :
       :
    (links n)
}
\end{verbatim}
which has the following topology:
\begin{verbatim}
  r - 0 - 1 - 2 - 3
           \
             4 - ... - n
\end{verbatim}

Branches can also be nested.  Together this structure is capable of specifying
all tree topologies.

{\Large \bf Link types for all versions.}

The library supports a number of "link" types that could correspond to a
constant transformation (with no dynamic properties), or a rigid body
connected to the previous link through a "joint" with some number of degrees
of freedom.  The possible link types supported in the various versions are
given below.  Tokens specific to a subset of the versions are given in the
comments following the pertinent lines.

A constant screw transformation about the z axis is given as follows
\begin{verbatim}
ZScrewTxLink {
    Name                             "some_label"      # v 2.1 and 3.0
    ZScrew_Parameters                d  theta
}
\end{verbatim}

A full six DOF "link" has been introduced in Version 3.0 which takes the
place of the DynamicRefMember (separate reference member specification has
been deprecated).  Use of this link only makes sense as the first link in a
particular tree.  Its state is specified only with respect to the System's
inertial reference system so placing it anywhere else in the configuration
essentially starts the topology over again:
\begin{verbatim}
MobilBaseLink {
    Name                             "some_label"      # v 2.1 and 3.0
    Graphics_Model                   "model_filename"  # v 3.0
    Graphics_Model_Index             i                 # v 2.0.3 and 2.1

    (rigid body parameters)

    Position                         x   y   z
    Orientation_Quat                 a_x a_y a_z  s
    Velocity                         w_x w_y w_z  v_x v_y v_z
}
\end{verbatim}
where specifications for rigid body parameters can be found below and the {\tt
Velocity} parameters specify the initial spatial (angular and linear) velocity
with respect to the inertial coordinate system.

A three DOF link (three orientation DOFs) is used to simulate a ball-in-socket
joint using Euler angles:
\begin{verbatim}
SphericalLink {
    Name                             "some_label"      # v 2.1 and 3.0
    Graphics_Model                   "model_filename"  # v 3.0
    Graphics_Model_Index             i                 # v 2.0.3 and 2.1

    (rigid body parameters)

    Position_From_Inboard_Link       x y z
    Initial_Joint_Angles             phi theta psi
    Initial_Angular_Velocity         w_x w_y w_z
    Axes_Limits                      phi theta psi
    Joint_Limit_Spring_Constant      K_limit
    Joint_Limit_Damper_Constant      B_limit
    Joint_Friction                   u
}
\end{verbatim}

A one DOF translational link is specified as follows:
\begin{verbatim}
PrismaticLink {
    Name                             "some_label"      # v 2.1 and 3.0
    Graphics_Model                   "model_filename"  # v 3.0
    Graphics_Model_Index             i                 # v 2.0.3 and 2.1

    (rigid body parameters)

    MDH_Parameters                   a  alpha  d  theta
    Initial_Joint_Velocity           qd
    Joint_Limits                     q_min q_max
    Joint_Limit_Spring_Constant      K_limit
    Joint_Limit_Damper_Constant      B_limit

    Joint_Friction                   u
}
\end{verbatim}

A one DOF rotational link is specified two different ways (with and without
motor model).  The version without a motor is specified as follows:
\begin{verbatim}
RevoluteLink {
    Name                             "some_label"      # v 2.1 and 3.0
    Graphics_Model                   "model_filename"  # v 3.0
    Graphics_Model_Index             i                 # v 2.0.3 and 2.1

    (rigid body parameters)

    MDH_Parameters                   a  alpha  d  theta
    Initial_Joint_Velocity           qd
    Joint_Limits                     q_min q_max
    Joint_Limit_Spring_Constant      K_limit
    Joint_Limit_Damper_Constant      B_limit

    Actuator_Type                    0
    Joint_Friction                   u
}
\end{verbatim}
The version with a motor is specified as follows:
\begin{verbatim}
RevoluteLink {
    Name                             "some_label"      # v 2.1 and 3.0
    Graphics_Model                   "model_filename"  # v 3.0
    Graphics_Model_Index             i                 # v 2.0.3 and 2.1

    (rigid body parameters)

    MDH_Parameters                   a  alpha  d  theta
    Initial_Joint_Velocity           qd
    Joint_Limits                     q_min q_max
    Joint_Limit_Spring_Constant      K_limit
    Joint_Limit_Damper_Constant      B_limit

    Actuator_Type                    1

    Motor_Torque_Constant            K_t
    Motor_BackEMF_Constant           K_b
    Motor_Armature_Resistance        R_a
    Motor_Inertia                    J_m
    Motor_Coulomb_Friction_Constant  u_c
    Motor_Viscous_Friction_Constant  u_v
    Motor_Max_Brush_Drop             V_max
    Motor_Half_Drop_Value            V_half
}
\end{verbatim}

The following is the order of the rigid body parameters:
\begin{verbatim}
    Mass                             m
    Inertia                          (3x3 matrix - 9 floats)
    Center_of_Gravity                cg_x cg_y cg_z

    (contact model parameters)
\end{verbatim}
In the case of hydrodynamic simulation, additional parameters are also
required resulting the following alternate form:
\begin{verbatim}
    Mass                             m
    Inertia                          (3x3 matrix - 9 floats)
    Center_of_Gravity                cg_x cg_y cg_z

    Volume                           V
    Added_Mass                       (6x6 matrix - 36 floats)
    Center_of_Bouyancy               cb_x cb_y cb_z
    Drag_Axis                        i (0=x, 1=y, 2=z)
    Cylinder_Bounds                  x0 xf
    Cylinder_Radius                  r
    Drag_Coefficient                 C_d

    (contact model parameters)
\end{verbatim}

The contact model parameters consist of the following:
\begin{verbatim}
    Number_of_Contact_Points   n
    Contact_Locations          x_1 y_1 z_1
                                    :
                               x_n y_n z_n
\end{verbatim}
Note that the {\tt Contact\_Locations} values are not needed if the {\tt
Number\_of\_Contact\_Points} is zero.
 */

const int FILENAME_SIZE = 256;

DMU_DLL_API dmSystem *dmuLoadFile_dm(char *filename);

/** Load an environment configuration file
 */
DMU_DLL_API dmEnvironment *dmuLoadFile_env(char *filename);

/** Find a dmObject with the given name
 */
DMU_DLL_API dmObject *dmuFindObject(const char *name, dmArticulation *system);

// functions from dmLoadFile.cpp
DMU_DLL_API bool readFilename(ifstream &cfg_ptr, char *filename);
DMU_DLL_API void readConfigParameterLabel(ifstream &cfg_ptr,
                                          const char *label);
DMU_DLL_API bool readConfigParameterLabelNonRecursive(ifstream &cfg_ptr,
                                          const char *label);

#endif
