#include <wx/wx.h>
#include <wx/sizer.h>
#include <wx/glcanvas.h>
#include "main.h"

 
// include OpenGL
#ifdef __WXMAC__
#include "OpenGL/glu.h"
#include "OpenGL/gl.h"
#else
#include <GL/glu.h>
#include <GL/gl.h>
#endif

// I still don't fully understand the mechanism behind idle func ...
// but the following works ... need to read more documentation
void BasicGLPane::updateSim(wxIdleEvent& event)
{
	if (!paused_flag)
	{
		for (int i=0; i<render_rate; i++)
	    {
			G_integrator->simulate(idt);
			sim_time += idt;
		}
	}
	camera->setPerspective(45.0, (GLfloat)getWidth()/(GLfloat)getHeight(), 1.0, 200.0);
	camera->update(mouse);
	camera->applyView();
	Refresh(); // ask for repaint

	// if you want the GL light to move with the camera, comment the following two lines - yiping
	GLfloat light_position0[] = { 1.0, 1.0, 1.0, 0.0 };
	glLightfv (GL_LIGHT0, GL_POSITION, light_position0);

	GLfloat light_position1[] = { -1.0, -1.0, 1.0, 0.0 };
	glLightfv (GL_LIGHT1, GL_POSITION, light_position1);


	timer_count++;
	dmGetSysTime(&tv);

	double elapsed_time = ((double) tv.tv_sec - last_tv.tv_sec) +
	                      (1.0e-9*((double) tv.tv_nsec - last_tv.tv_nsec));

	if (elapsed_time > 2.5)
	{
		rtime += elapsed_time;
		cerr << "time/real_time: " << sim_time << '/' << rtime
		   << "  frame_rate: " << (double) timer_count/elapsed_time << endl;

		timer_count = 0;
		last_tv.tv_sec = tv.tv_sec;
		last_tv.tv_nsec = tv.tv_nsec;
	}

	// WakeUpIdle(); //?
	event.RequestMore();// render continuously, not only once on idle
}
