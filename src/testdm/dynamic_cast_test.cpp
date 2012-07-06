/*****************************************************************************
 *     File: dynamic_cast_test.cpp
 *   Author: YL
 *  Created: 2012 Jul 3
 *  Summary: Test for dynamic_cast usage
 *****************************************************************************/
#if defined(__APPLE__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <dm.h>
#include <dmu.h>         
#include <dmSystem.hpp>      
#include <dmRigidBody.hpp>
#include <dmArticulation.hpp>
#include <dmLink.hpp>
#include <dmForce.hpp>
#include <dmContactModel.hpp>
#include <dmRevoluteLink.hpp>
#include <dmMobileBaseLink.hpp>


dmArticulation *G_robot;




int main(int argc, char** argv)
{

	glutInit(&argc, argv);
	char *filename = "carts.cfg";
	glutInitWindowSize(640, 480);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutCreateWindow("dynamic_cast_test");	

	ifstream cfg_ptr;
	cfg_ptr.open(filename);

	char robot_flname[FILENAME_SIZE]; // 256 defined in dmu.h
	readConfigParameterLabel(cfg_ptr,"Robot_Parameter_File");
	readFilename(cfg_ptr, robot_flname);
	cout << robot_flname << endl;
	G_robot = dynamic_cast<dmArticulation*>(dmuLoadFile_dm(robot_flname));
   
	for (unsigned int i =0; i< G_robot->getNumLinks(); i++)
	{
		dmLink* link;
		dmForce* force;
		link = G_robot->getLink(i);
		cout<<"link "<<i<<" DOF : "<<link->getNumDOFs()<<"   ";
		dmRigidBody* rb;
		rb = dynamic_cast<dmRigidBody*>(link);
		if (rb != NULL)
		{
			force = rb->getForce(0);
			unsigned int nF = rb->getNumForces();
			cout<<"is RigidBody, "<<"  ";
		
			for (unsigned int j =0; j< nF; j++)
			{
				dmContactModel* cf;
				cf = dynamic_cast<dmContactModel*>(rb->getForce(j));
				if (cf != NULL)
				{
					cout<<"f_"<<j<<" is contact_f ";
				}
				else
				{
					cout<<"NNN  ";
				}
			}
			cout<<endl;
		} 
		else
			cout<<"NULL  "<<endl;
	}

	return 0;             
}
