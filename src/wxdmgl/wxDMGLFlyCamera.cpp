/*****************************************************************************
 *     File: wxDMGLFlyCamera.cpp
 *   Author:  
 *  Created: Jun 12, 2012
 *  Summary:
 *****************************************************************************/

#include "wxDMGLFlyCamera.hpp"

const float DEGTORAD = (float)(M_PI/180.0);

//============================================================================
// class wxDMGLFlyCamera : public wxDMGLCamera
//============================================================================


wxDMGLFlyCamera::wxDMGLFlyCamera() : wxDMGLCamera()
{
   reset();
}



wxDMGLFlyCamera::~wxDMGLFlyCamera()
{
#ifdef DEBUG
   cerr << "destructing wxDMGLFlyCamera" << endl;
#endif
}



void wxDMGLFlyCamera::reset()
{
   wxDMGLCamera::reset();

   m_pos_ff[0] = m_pos_ff[1] = m_pos_ff[2] = 0.0;
   setAzimuth(0.0);
   setElevation(0.0);
   m_trans_scale = 1.0;
}


void wxDMGLFlyCamera::outputState()
{
   cerr << "wxDMGLFlyCamera state:" << endl;
   cerr << "Azimuth:    " << m_azimuth << endl;
   cerr << "Elevation:  " << m_elevation << endl;
   cerr << "Camera pos: " << m_pos_ff[0] << ", "
                          << m_pos_ff[1] << ", "
                          << m_pos_ff[2] << endl;
}


void wxDMGLFlyCamera::updateAzimuth(float delta_azimuth)
{
   setAzimuth(m_azimuth + delta_azimuth);
}


void wxDMGLFlyCamera::setAzimuth(float new_azimuth)
{
   m_azimuth = new_azimuth;

   // allow it to wrap around
   while (m_azimuth > 180.0)
   {
      m_azimuth -= 360.0;
   }
   while (m_azimuth < -180.0)
   {
      m_azimuth += 360.0;
   }

   double tmp = m_azimuth*DEGTORAD;
   m_sin_az = sin(tmp);
   m_cos_az = cos(tmp);
}


void wxDMGLFlyCamera::updateElevation(float delta_elevation)
{
   setElevation(m_elevation + delta_elevation);
}


void wxDMGLFlyCamera::setElevation(float elevation)
{
   m_elevation = elevation;

   // allow it to wrap around
   while (m_elevation > 180.0)
   {
      m_elevation -= 360.0;
   }
   while (m_elevation < -180.0)
   {
      m_elevation += 360.0;
   }

   double tmp = m_elevation*DEGTORAD;
   m_sin_el = sin(tmp);
   m_cos_el = cos(tmp);
}


void wxDMGLFlyCamera::updateFlyPos(float delta_pos[3])
{
   m_pos_ff[0] += delta_pos[0];
   m_pos_ff[1] += delta_pos[1];
   m_pos_ff[2] += delta_pos[2];
}

void wxDMGLFlyCamera::updateFlyPos(float delta_x, float delta_y, float delta_z)
{
   m_pos_ff[0] += delta_x;
   m_pos_ff[1] += delta_y;
   m_pos_ff[2] += delta_z;
}





void wxDMGLFlyCamera::update(wxDMGLMouse *mouse)
{
   static float delta_pos[3];

   if (mouse->in_canvas_flag)
   {
      if (mouse->button_flags & (MOUSE_L_DN|
                                 MOUSE_M_DN|
                                 MOUSE_R_DN))
      {
         if (mouse->button_flags & MOUSE_L_DN)
         {
            // mouse x position controls azimuth
            // mouse y position controls elevation
            updateAzimuth(2.0*mouse->xchan);
            updateElevation(2.0*mouse->ychan);
         }

         delta_pos[0] = delta_pos[1] = delta_pos[2] = 0.0;

         if (mouse->button_flags & MOUSE_M_DN)
         {
            delta_pos[0] = 2.0*m_trans_scale*mouse->xchan;
            delta_pos[2] = 2.0*m_trans_scale*mouse->ychan;
         }

         if (mouse->button_flags & MOUSE_R_DN)
         {
            delta_pos[1] = -2.0*fabs(mouse->ychan)*mouse->ychan;
         }

         m_pos_ff[0] += delta_pos[0]*m_cos_az
                      - delta_pos[1]*m_sin_az*m_cos_el
                      + delta_pos[2]*m_sin_az*m_sin_el;

         m_pos_ff[1] += delta_pos[0]*m_sin_az
                      + delta_pos[1]*m_cos_az*m_cos_el
                      - delta_pos[2]*m_cos_az*m_sin_el;

         m_pos_ff[2] += delta_pos[1]*m_sin_el
                      + delta_pos[2]*m_cos_el;
      }
   }
}


void wxDMGLFlyCamera::applyView()
{
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();

   m_look_at[0] =  m_sin_az*m_cos_el;
   m_look_at[1] =  m_sin_el;
   m_look_at[2] = -m_cos_az*m_cos_el;

   m_up[0] = -m_cos_az;
   m_up[1] = 0.0;
   m_up[2] = -m_sin_az;

   gluLookAt(m_pos_ff[0],
             m_pos_ff[1],
             m_pos_ff[2],
             m_pos_ff[0] + m_look_at[0],
             m_pos_ff[1] + m_look_at[1],
             m_pos_ff[2] + m_look_at[2],
             m_up[0], m_up[1], m_up[2]);
}
