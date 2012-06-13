/*****************************************************************************
 *     File: wxDMGLFlyCamera.hpp
 *   Author: 
 *  Created: Jun 12, 2012
 *  Summary: free-fly camera sub-class definition
 *****************************************************************************/

#ifndef _WXDMGL_FLY_CAMERA_HPP
#define _WXDMGL_FLY_CAMERA_HPP

#include "wxDMGL.h"
#include "wxDMGLCamera.hpp"
#include "wxDMGLMouse.hpp"

class WXDMGL_DLL_API wxDMGLFlyCamera : public wxDMGLCamera
{
public:
   wxDMGLFlyCamera();
   virtual ~wxDMGLFlyCamera();

   void reset();
   void outputState();
   void setTranslationScale(float scale) {m_trans_scale = scale;}

   // Lower level functions to affect camera state changes
   void updateAzimuth(float delta_azimuth);
   void updateElevation(float delta_elevation);
   void updateFlyPos(float delta_pos_ff[3]);
   void updateFlyPos(float delta_x, float delta_y, float delta_z);

   void setAzimuth(float azimuth);
   void setElevation(float elevation);
   void setFlyPos(float pos_ff[3]);
   void setFlyPos(float x, float y, float z);

   //   update(mouse *) 
   //   changes the camera position and orientation
   //   according to the mouse position relative to the screen and
   //   which mouse buttons are being pressed, and computes the new
   //   matrix from these values:
   //            - the camera can be moved right/left and up/down with
   //              the MIDDLE mouse button and the distance
   //              from the center of the window using
   //            - the camera can be moved forward/backward using the RIGHT
   //              mouse button and the vertical distance from the center of
   //              the window
   //            - the yaw and pitch can be modified using the LEFT mouse
   //              button and the horizontal and vertical distances from the
   //              center of the window respectively.
   //
   // NOTE call applyView() to push changes to the view matrix.
   void update(wxDMGLMouse *mouse);

   void applyView();

private:
   float m_pos_ff[3];                        // current camera position
   float m_look_at[3], m_up[3];
   float m_elevation, m_sin_el, m_cos_el;    // elevation angle in degrees
   float m_azimuth,   m_sin_az, m_cos_az;    // azimuth angle in degrees

   float m_trans_scale;

   wxDMGLFlyCamera(const wxDMGLFlyCamera&);      // copy constructor forbidden
};

#endif
