/*****************************************************************************
 *     File: wxDMGLPolarCamera.cpp
 *   Author: 
 *  Created: Jun 12, 2012
 *  Summary:
 *****************************************************************************/

#include "wxDMGL.h"
#include "wxDMGLMouse.hpp"
#include "wxDMGLCamera.hpp"
#include "wxDMGLPolarCamera.hpp"

const float DEGTORAD = (float)(M_PI/180.0);

//============================================================================
// class wxDMGLPolarCamera : public wxDMGLCamera
//============================================================================


wxDMGLPolarCamera::wxDMGLPolarCamera() :  wxDMGLCamera()
{
   reset();
}

wxDMGLPolarCamera::~wxDMGLPolarCamera()
{
#ifdef DEBUG
   cerr << "destructing wxDMGLPolarCamera" << endl;
#endif
}


void wxDMGLPolarCamera::reset()
{
   wxDMGLCamera::reset();//?

   m_pos_coi[0] = m_pos_coi[1] = m_pos_coi[2] = 0.0;
   setAzimuth(0.0);
   setElevation(0.0);
   setRadius(0.5);
   m_trans_scale = 1.0;
}



void wxDMGLPolarCamera::outputState()
{
   cerr << "wxDMGLPolarCamera state:" << endl;
   cerr << "Azimuth:   " << m_azimuth << endl;
   cerr << "Elevation: " << m_elevation << endl;
   cerr << "Radius:    " << m_radius << endl;
   cerr << "COI pos:   " << m_pos_coi[0] << ", "
                         << m_pos_coi[1] << ", "
                         << m_pos_coi[2] << endl;
}

//----------------------------------------------------------------------------
//   Function: updateAzimuth
//    Summary: Change the azimuth angle of the camera 
// Parameters: change in azimuth angle, in degrees
//    Returns: none
//----------------------------------------------------------------------------
void wxDMGLPolarCamera::updateAzimuth(float delta_azimuth)
{
   setAzimuth(m_azimuth + delta_azimuth);
}


//----------------------------------------------------------------------
//   Function: setAzimuth
//    Summary: Set the azimuth angle of the camera 
// Parameters: azimuth angle, in degrees
//    Returns: none
//----------------------------------------------------------------------
void wxDMGLPolarCamera::setAzimuth(float new_azimuth)
{
   m_azimuth = new_azimuth;

   // allow it to wrap around
   if (m_azimuth > 180.0)
   {
      m_azimuth -= 360.0;
   }
   else if (m_azimuth < -180.0)
   {
      m_azimuth += 360.0;
   }

   double tmp = m_azimuth*DEGTORAD;
   m_sin_az = sin(tmp);
   m_cos_az = cos(tmp);
}

//----------------------------------------------------------------------------
//   Function: updateElevation
//    Summary: Change the elevation angle of the camera 
// Parameters: change in elevation angle, in degrees
//    Returns: none
//----------------------------------------------------------------------------
void wxDMGLPolarCamera::updateElevation(float delta_elevation)
{
   setElevation(m_elevation + delta_elevation);
}

//----------------------------------------------------------------------
//   Function: setElevation
//    Summary: set the elevation angle of the camera 
// Parameters: elevation angle, in degrees
//    Returns: none
//----------------------------------------------------------------------
void wxDMGLPolarCamera::setElevation(float new_elevation)
{
   m_elevation = new_elevation;
   if (m_elevation > 90.0)         // looking straight down from overhead
      m_elevation = 90.0;
   else if (m_elevation < -90.0)   // straight up from underneath
      m_elevation = -90.0;

   double tmp = m_elevation*DEGTORAD;
   m_sin_el = sin(tmp);
   m_cos_el = cos(tmp);
}

//----------------------------------------------------------------------------
//   Function: updateRadius
//    Summary: Change the distance of the camera from the center of
//             interest.  
// Parameters: change in distance to robot, in meters
//    Returns: none
//----------------------------------------------------------------------------
void wxDMGLPolarCamera::updateRadius(float delta_radius)
{
   setRadius(m_radius + delta_radius);
}

//----------------------------------------------------------------------------
//   Function: setRadius
//    Summary: Set the distance of the camera from the center of
//             interest.  
// Parameters: distance to robot, in meters
//    Returns: none
//----------------------------------------------------------------------------
void wxDMGLPolarCamera::setRadius(float new_radius)
{
   m_radius = new_radius;
   if (m_radius < 0.001f)           // don't let it go negative or zero
   {
      m_radius = 0.001f;
   }
}

//----------------------------------------------------------------------------
//   Function: setCOI
//    Summary: Change the position of the camera wrt to world
//             coordinate system.
// Parameters: 3D vector specifying the position change
//    Returns: none
//----------------------------------------------------------------------------
void wxDMGLPolarCamera::updateCOI(float delta_coi[3])
{
   m_pos_coi[0] += delta_coi[0];
   m_pos_coi[1] += delta_coi[1];
   m_pos_coi[2] += delta_coi[2];
}

//----------------------------------------------------------------------------
//   Function: setCOI
//    Summary: set absolute position of the camera wrt to world/Performer
//             coordinate system.
// Parameters: 3D vector specifying the new position
//    Returns: none
//----------------------------------------------------------------------------
void wxDMGLPolarCamera::setCOI(float pos_coi[3])
{
   m_pos_coi[0] = pos_coi[0];
   m_pos_coi[1] = pos_coi[1];
   m_pos_coi[2] = pos_coi[2];
}

//----------------------------------------------------------------------------
//   Function: setCOI
//    Summary: set absolute position of the camera wrt to world/Performer
//             coordinate system.
// Parameters: 3-tuple of floats
//    Returns: none
//----------------------------------------------------------------------------
void wxDMGLPolarCamera::setCOI(float x, float y, float z)
{
   m_pos_coi[0] = x;
   m_pos_coi[1] = y;
   m_pos_coi[2] = z;
}




void wxDMGLPolarCamera::update(wxDMGLMouse *mouse)
{
   //!!
   static int first_time = 1;
   static int last_button = 0x0;
   static int last_pos[2], delta[2] = {0,0};
	delta[0] = 0;
	delta[1] = 0;
   if (mouse->in_canvas_flag)
   {
      if (mouse->button_flags & (MOUSE_L_DN|
                                 MOUSE_M_DN|
                                 MOUSE_R_DN))
      {
         if (first_time)
         {
            last_pos[0] = mouse->xwin;
            last_pos[1] = mouse->ywin;
            first_time = 0;
         }

         delta[0] = mouse->xwin - last_pos[0];
         delta[1] = mouse->ywin - last_pos[1];

         last_button = mouse->button_flags;
      }
      last_pos[0] = mouse->xwin;
      last_pos[1] = mouse->ywin;
   }

   // change viewpoint if delta is non-zero
   if ((delta[0]*delta[0] + delta[1]*delta[1]) > 0.0)
      spinScene(delta, last_button);
}
