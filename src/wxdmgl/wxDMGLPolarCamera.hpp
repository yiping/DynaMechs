/*****************************************************************************
 *     File: wxDMGLPolarCamera.hpp
 *   Author: 
 *  Created: Jun 12, 2012
 *  Summary:
 *****************************************************************************/

#ifndef _WXDMGL_POLAR_CAMERA_HPP
#define _WXDMGL_POLAR_CAMERA_HPP

#include "wxDMGL.h"
#include "wxDMGLMouse.hpp"
#include "wxDMGLCamera.hpp"

class WXDMGL_DLL_API wxDMGLPolarCamera : public wxDMGLCamera
{
public:
   wxDMGLPolarCamera();
   virtual ~wxDMGLPolarCamera();

   // return to the initial state.
   void reset();
   void outputState();
   void setTranslationScale(float scale) {m_trans_scale = scale;}

   // Lower level functions to affect the same sorts of changes.
   void updateAzimuth(float delta_azimuth);
   void updateElevation(float delta_elevation);
   void updateRadius(float delta_radius);
   void updateCOI(float delta_coi[3]);

   void setAzimuth(float new_azimuth);
   void setElevation(float new_elevation);
   void setRadius(float new_radius);
   void setCOI(float pos_coi[3]);
   void setCOI(float x, float y, float z);


   //   update(mouse *) 
   //   changes the camera position and orientation
   //   according to the mouse position relative to the screen and
   //   which mouse buttons are being pressed, and computes the new
   //   matrix from these values:
   //   -- the coi position can be moved in the current
   //      view plane with the MIDDLE mouse button and the distance
   //      from the center of the window using
   //   -- the radius can be modified using the RIGHT mouse
   //      button and the vertical distance from the center of the
   //      window
   //   -- the heading and pitch can be modified using the LEFT mouse
   //      button and the horizontal and vertical distances from the
   //      center of the window respectively.
   //
   // Call applyView() to push changes to the view matrix.

   virtual void spinScene(int delta[2], int button_flags) = 0;
   virtual void update(wxDMGLMouse *mouse);
   virtual void applyView() {};

protected:
   float m_pos_coi[3];  // lookat position for center of interest

   float m_radius;      // distance from center of interest
   float m_elevation, m_sin_el, m_cos_el;  // elevation angle in degrees
   float m_azimuth,   m_sin_az, m_cos_az;  // azimuth angle in degrees

   float m_trans_scale;

   float m_look_from[3], m_up[3];  // for the gluLookAt command

   wxDMGLPolarCamera(const wxDMGLPolarCamera&);  // copy constructor nono
};

#endif
