/*****************************************************************************
 *     File: wxDMGLPolarCamera_zup.cpp
 *   Author: 
 *  Created: Jun 12, 2012
 *  Summary:
 *****************************************************************************/

#include "wxDMGL.h"
#include "wxDMGLPolarCamera.hpp"
#include "wxDMGLPolarCamera_zup.hpp"


wxDMGLPolarCamera_zup::wxDMGLPolarCamera_zup() :
      wxDMGLPolarCamera()
{
}



void wxDMGLPolarCamera_zup::spinScene(int delta[2], int button)
{
   if (button & MOUSE_L_DN)
   {
      // mouse x position controls azimuth
      // mouse y position controls elevation
      updateAzimuth(-delta[0]*0.5);
      updateElevation(-delta[1]*0.5);
   }

   if (button & MOUSE_M_DN)
   {
      static float delta_pos[3];

      delta_pos[0] =-m_trans_scale*delta[0];
      delta_pos[1] = m_trans_scale*delta[1];
      //delta_pos[2] =  0.0;

      m_pos_coi[0] += delta_pos[0]*m_cos_az
                   + delta_pos[1]*m_sin_az*m_sin_el;
                 //+ delta_pos[2]*m_sin_az*m_cos_el;

      m_pos_coi[1] += delta_pos[0]*m_sin_az
                   - delta_pos[1]*m_cos_az*m_sin_el;
                 //- delta_pos[2]*m_cos_az*m_cos_el;

      m_pos_coi[2] += delta_pos[1]*m_cos_el;
                 //- delta_pos[2]*m_sin_el;

   }

   if (button & MOUSE_R_DN)
   {
      updateRadius(-m_trans_scale*delta[1]);
   }
}



void wxDMGLPolarCamera_zup::applyView()
{
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();

   m_look_from[0] = m_pos_coi[0] + m_radius*m_sin_az*m_cos_el;
   m_look_from[1] = m_pos_coi[1] - m_radius*m_cos_az*m_cos_el;
   m_look_from[2] = m_pos_coi[2] - m_radius*m_sin_el;

   m_up[0] = m_sin_az*m_sin_el;
   m_up[1] =-m_cos_az*m_sin_el;
   m_up[2] = m_cos_el;

   //cout<<"zup - gluLookAt()"<<endl;
   //cout<<"look from: ["<<m_look_from[0]<<" "<<m_look_from[1]<<" "<<m_look_from[2]<<"]"<<endl;
   //cout<<"      up: ["<<m_up[0]<<" "<<m_up[1]<<" "<<m_up[2]<<"]"<<endl;
   //cout<<"p_coi: ["<<m_pos_coi[0]<<" "<<m_pos_coi[1]<<" "<<m_pos_coi[2]<<"]"<<endl;
   gluLookAt(m_look_from[0], m_look_from[1], m_look_from[2],
             m_pos_coi[0], m_pos_coi[1], m_pos_coi[2],
             m_up[0], m_up[1], m_up[2]);

}
