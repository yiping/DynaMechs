/*
 *  SlipModel.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 1/8/13.
 *  Copyright 2013 __MyCompanyName__. All rights reserved.
 *
 */
#define F_SIZE 2

#include "SlipModel.h"
#include <Eigen/Dense>
#include "dmTime.h"
#include "GlobalFunctions.h"

void SlipModel::integrate(Float dt)
{
	dynamics();
	pos += vel*dt;
	vel += acc*dt;
	time +=dt;
}
void SlipModel::dynamics()
{
	Vector2F relPos = pos-anchor;
	length = relPos.norm();
	relPos /= length;
	
	acc = springConst * (restLength-length)*relPos / mass;
	acc(1)-=9.8;
}

void SlipModel::findParams()
{

	
	
	



}
void SlipModel::optimize()
{
	/*VectorXF x(2);
	x << -15.5,17;
	MatrixXF Jac(2,2);
	MatrixXF H(2,2);
	VectorXF grad(2);
	VectorXF fx(2);
	Float obj = 1000;
	VectorXF d(2);*/
	

	mass=72.5748; 
	restLength=1.0;
	
	tContactDes = 0.155920276;
    tFlightDes = 0.141643052;
	
	VectorXF x(3);
	x << 0.5317, -0.5941, 1.7959;
	//x << .19, -1.2250, 6.6683;
	MatrixXF Jac(4,3);
	MatrixXF H(3,3);
	VectorXF grad(3);
	VectorXF fx(4);
	Float obj = 1000;
	VectorXF d(3);
	
	
	
	evalResidual(x, fx, obj);
	
	cout << "res " << obj << endl;
	cout << "fx " << fx.transpose() << endl;
	//exit(-1);
	
	while (obj > 1e-10) {
		evalJac(x,  fx, obj, Jac, grad);
	//cout << "Jac = " << Jac << endl;
	//cout << "grad = " << grad.transpose() << endl;
		//H.setIdentity();
		//H*=.001;
		H = Jac.transpose()*Jac;
		FullPivHouseholderQR<MatrixXF> lu(H);
		
		d = lu.solve(-grad);
	//cout << "d = " << d.transpose() << endl;
		armijoStep(x, d, obj, fx, grad);
	}
	cout << x << endl;
}


void SlipModel::simulatePeriod(const VectorXF& ps)
{
	dmTimespec t1, t2;
	dmGetSysTime(&t1);
	
	const Float g = 9.8;
	
	Float tdAngle = ps(0);
	Float vy   = ps(1);
	Float vx = 5.5;
	
	pos << -restLength * sin(tdAngle), restLength*cos(tdAngle);
	vel << vx, vy;
	anchor << 0,0;
	springConst = ps(2)*1e4;
	time = 0;
	length = restLength;
	
	const Float idt = .00008;
	
	while ((length < restLength || vel(1)<0) && time < 1.) {
		if ((restLength - length) < .01 ) {
			dynamics();
			pos += vel*idt;
			if (pos.norm() > restLength) {
				pos -= vel*idt;
				Float pdotv = pos.dot(vel);
				//cout << "p " << pos.transpose() << endl;
				//cout << "pn " << pos.norm() << endl;
				//cout << "v " << vel.transpose() << endl;
				Float newDt = (2*pdotv - sqrt(4*pdotv*pdotv+4*vel.dot(vel)*(restLength*restLength-pos.dot(pos))))/(2*vel.dot(vel));
				//cout << "newdt " << newDt << endl;
				pos += vel*newDt;
				vel += acc*newDt;
				time+= newDt;
				//cout << "here " << endl;
				//cout << pos.norm() - restLength;
				//exit(-1);
				break;
			}
			else {
				vel += acc*idt;
				time +=idt;
			}
		}
		else {
			integrate(idt);
		}
	}
	dmGetSysTime(&t2);
	//cout <<"Time! "<<
	//timeDiff(t1, t2) << endl; 
	
	//cout << "final state " << endl;
	//cout << "Pos " << pos.transpose() << endl;
	//cout << "Vel " << vel.transpose() << endl;
	//cout << "t " << time << endl;
	
	tContact = time;
	Float vOverG = vel(1)/g;
	tFlight = vOverG + sqrt(vOverG*vOverG-2*(restLength*cos(tdAngle)-pos(1))/g);
	
}
void SlipModel::evalResidual(const VectorXF& ps, VectorXF &fs, Float & o)
{
	//const Float p0 = ps(0);
	//const Float p1 = ps(1);
	//fs << p0*p0+3*p1-6+cos(p1)*p0 , 5*p0-p1*p1*p0-1;
	simulatePeriod(ps);
	fs << vel(0) - 5.5, (vel(1)-9.8*tFlight)-ps(1), tContact-tContactDes, tFlight-tFlightDes;
	
	o = .5 * fs.dot(fs);
}


void SlipModel::evalJac(const VectorXF& ps, const VectorXF & f0, const Float & obj0, MatrixXF & J, VectorXF &g)
{
	Float dx = 1e-4;
	
	const int pSize = ps.size();
	const int fSize = f0.size();
	
	Float objTest;
	VectorXF pTest = ps;
	VectorXF fTest(fSize);
	
	for (int i=0; i<pSize; i++) {
		pTest(i) += dx;
		evalResidual(pTest, fTest, objTest);
		J.col(i) = (fTest-f0)/dx;
		pTest(i) -= dx;
	}
	g = J.transpose()*f0;
}

void SlipModel::armijoStep(VectorXF&x,const VectorXF& d, Float & obj, VectorXF &f, const VectorXF& g0)
{
	const Float s = 1.0;
	const Float beta = .5;
	const Float sigma = .01;
	const Float slope = sigma*g0.dot(d);
	
	Float alpha = s;
	
	//cout << "slope init = " << slope << endl;
	
	Float objTest;
	VectorXF fTest(f.size());
	VectorXF xTest = x+alpha*d;
	evalResidual(xTest, fTest, objTest);
	
	//cout << "d " << d.transpose() << endl;
	//cout << "xTest " << xTest.transpose() << endl;
	//cout << "new = " << objTest << endl;
	//cout << "fxTest = " << fTest.transpose() << endl;
	
	int inIter = 1;
	while (((obj - objTest) < -alpha*slope) || isnan(objTest)) {
		alpha*=beta;
		xTest = x+alpha*d;
		evalResidual(xTest, fTest, objTest);
		//cout << "new = " << objTest << endl;
		inIter ++;
	}
	x=xTest;
	f=fTest;
	obj=objTest;
	
	cout << objTest << "  \tinIter = " << inIter << endl; 
}



