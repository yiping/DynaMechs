/*
 *  CustomTaskSpaceController.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 4/23/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */
//#define EIGEN_NO_DEBUG
#include "CustomTaskSpaceController.h"
#include "dmContactModel.hpp"
#include "dmRigidBody.hpp"
#include "mosek.h"
#include "GlobalDefines.h"
#include <Eigen/Cholesky>


//#define OPTIM_DEBUG



CustomTaskSpaceController::CustomTaskSpaceController(dmArticulation * art) : TaskSpaceController(art) {
	
	SupportJacobians.resize(NS);
	
	// Initialize Support Jacobians
	for (int i=0; i<NS; i++) {
		SupportJacobians[i] = MatrixXF::Zero(6,NP+6);;
	}
	
	tauLb.resize(NJ);
	tauUb.resize(NJ);
	fnetUb.resize(NS);
}

//----------------------------------------------------------------------------

void CustomTaskSpaceController::ObtainArticulationData() {
	artic->computeH();
	artic->computeCandG();
	
	for (int i=0; i<NS; i++) {
		artic->computeJacobian(SupportIndices[i], SupportXforms[i],SupportJacobians[i]);
	}
}

void CustomTaskSpaceController::UpdateTauObjective() {
	
}

void CustomTaskSpaceController::UpdateObjective() {
}

void CustomTaskSpaceController::UpdateVariableBounds() {
	// Initialize Bounds for Tau Variables
	for (int i=0; i<NJ; i++) {
		tauLb(i) = -500;
		tauUb(i) = 500;
	}
}

void CustomTaskSpaceController::AssignFootMaxLoad(int index, double maxLoad) {
	fnetUb[index]=maxLoad;	
}



void CustomTaskSpaceController::UpdateInitialConstraintBounds() {
	fnetUb.setConstant(MSK_INFINITY);
}

void CustomTaskSpaceController::UpdateHPTConstraintBounds() {
	int k=0;
	numConstraintTasks= 0, numOptimTasks = 0;
	
	for (int i=0; i<taskConstrActive.size(); i++) {
		if (taskConstrActive(i) > 0) {
			numConstraintTasks ++;
		}
		else if (taskOptimActive(i) >0) {
			numOptimTasks++;
		}
	}
	Lttau.resize(numOptimTasks,NJ);
	Lctau.resize(numConstraintTasks,NJ);
	
	Ltf.resize(numOptimTasks,3*NS*NP);
	Lcf.resize(numConstraintTasks,3*NS*NP);
	
	bt.resize(numOptimTasks);
	bc.resize(numConstraintTasks);
	//TaskWeight.setOnes();
	int conCount = 0, optimCount = 0;
	for (int i=0; i<taskConstrActive.size(); i++) {
		if (taskConstrActive(i) > 0) {
			Lctau.row(conCount) = LambdaInvTau.row(i);
			Lcf.row(conCount)   = LambdaInvF.row(i);
			bc(conCount) = TaskBias(i)+eBiasCandG(i);
			conCount++;
		}
		else if (taskOptimActive(i) >0) {
			Lttau.row(optimCount) = TaskWeight(i)*LambdaInvTau.row(i);
			Ltf.row(optimCount)   = TaskWeight(i)*LambdaInvF.row(i);
			bt(optimCount) = TaskWeight(i)*(TaskBias(i)+eBiasCandG(i));
			optimCount++;
		}
	}
	
	fLen = 3*NP*NS;
	xLen = NJ+3*NP*NS + numOptimTasks + 1;  
	vLen = numOptimTasks+numConstraintTasks;
	
	barLen = 2*NJ + NP*NS + 1;
	fStart = NJ;
	eStart = NJ + 3*NS*NP;
	zStart = eStart+numOptimTasks;
	
	grads.resize(barLen);
	Hs.resize(barLen);
	iHs.resize(barLen);
	rdInit.setZero(xLen);
	rdInit(xLen-1) = 1;
	
	
	int barInd = 0;
	for (int i=0; i<NS*NP; i++) {
		grads[barInd].resize(3);
		Hs[barInd].setZero(3);
		
		Hs[barInd].diagonal() << 2,2,-2;
		//Hs[barInd](1) = 2;
		//Hs[barInd](2) = -2;
		
		iHs[barInd].setZero(3);
		iHs[barInd].diagonal()<< .5,.5,-.5;
		barInd++;
	}
	grads[barInd].resize(numOptimTasks+1);
	VectorXF vec(numOptimTasks+1);
	vec.setConstant(2);
	vec(numOptimTasks) = -2;
	
	//Hs[barInd] = MatrixXF::Identity(numOptimTasks+1,numOptimTasks+1)*2;
	Hs[barInd].resize(numOptimTasks+1);
	Hs[barInd].diagonal() = vec;
	
	//Hs[barInd](numOptimTasks,numOptimTasks) = -2;
	vec.setConstant(.5);
	vec(numOptimTasks) = -.5;
	iHs[barInd].resize(numOptimTasks+1);
	iHs[barInd].diagonal() = vec;
	//iHs[barInd] = MatrixXF::Identity(numOptimTasks+1,numOptimTasks+1)*.5;
	//iHs[barInd](numOptimTasks,numOptimTasks) = -.5;
	barInd++;
	
	
}



void CustomTaskSpaceController::UpdateConstraintMatrix() {
	Hdecomp.compute(artic->H);
	MatrixXF invHJt = Hdecomp.solve(TaskJacobian.transpose());
	MatrixXF JinvH = invHJt.transpose();
	int m = TaskJacobian.rows();
	
	LambdaInvTau = JinvH.block(0,6,m,NJ);
	LambdaInvTau.topRows(6).setZero();
	
	LambdaInvF.setZero(m,3*NS*NP);
	
	eBiasCandG.resize(m);
	eBiasCandG = TaskJacobian*Hdecomp.solve(artic->CandG);
	
	// Compute the LamndaInvF using one leg at a time, exploiting common structure.
	for (int i = 0; i<NS; i++) {
		MatrixXF LambdaInvFLoc(m,6);
		LambdaInvFLoc = JinvH * SupportJacobians[i].transpose();
		
		for (int j=0; j<NP; j++) {
			LambdaInvF.block(0,3*(NP*i+j),m,3) = LambdaInvFLoc*PointForceXforms[i][j].rightCols(3);
			// Mu scaling
			LambdaInvF.col(3*(NP*i+j)+2) /= MU;
		}
	} 
}
Float CustomTaskSpaceController::norm(const VectorXF & a,const VectorXF & b)
{
	return sqrt(pow(a.norm(),2) + pow(b.norm(),2));
}


void CustomTaskSpaceController::logBarrier(const VectorXF & x, Float &b, VectorXF & bs)
{	
	int barInd = 0;
	
	bs.resize(barLen + NS*NP+1);
	
	for (int i=0; i<NS*NP; i++) {
		bs(barInd++) = pow(x(NJ+3*i+2),2)-pow(x(NJ+3*i+1),2)-pow(x(NJ+3*i+0),2);
	}
	bs(barInd++) = pow(x(zStart),2)-pow(x.segment(eStart,numOptimTasks).norm(),2);
	
	
	for (int i=0; i<NJ; i++) {
		bs(barInd++) = tauUb(i)-x(i);
		bs(barInd++) = x(i) - tauLb(i);
	}
	
	for (int i=0; i<NS*NP; i++) {
		bs(barInd++) = x(NJ+3*i+2);
	}
	bs(barInd++) = x(zStart);
	
	b=0;
	for (int i=0; i<barLen; i++) {
		b-=log(bs(i));
	}
}

void CustomTaskSpaceController::evalBarrier(const VectorXF & x, bool computeHess, VectorXF & bs, VectorXF& grad, MatrixXF& H, MatrixXF & iH)
{	
#define H_USED	
	grad.setZero(xLen);
	if (computeHess) {
#ifdef H_USED		
		H.setZero(xLen,xLen);
#endif
		iH.setZero(xLen,xLen);
	}
	Float b;
	logBarrier(x,b,bs);
		
	int barInd = 0;

	Vector3F y;
	VectorXF ye(numOptimTasks+1);
	
	int fInd = NJ;
	for (int i=0; i<NS*NP; i++) {
		grads[barInd] << 2*x(fInd), 2*x(fInd+1), -2*x(fInd+2);
		Float invBar = 1/bs(barInd);
		
		grad.segment(fInd,3) = invBar*grads[barInd];
		
		if (computeHess) {
#ifdef H_USED			
			H.block(fInd,fInd,3,3)=invBar*invBar * grads[barInd]*grads[barInd].transpose();
			H.block(fInd,fInd,3,3).diagonal()+= invBar*Hs[barInd].diagonal();
#endif
			y = iHs[barInd] * grads[barInd];
			//iH.block(fInd,fInd,3,3)=bs(barInd) * iHs[barInd] - y*y.transpose()/(1+grads[barInd].dot(y)/bs(barInd));
			iH.block(fInd,fInd,3,3)=- y*y.transpose()/(1+grads[barInd].dot(y)*invBar);
			
			iH.block(fInd,fInd,3,3).diagonal()+=bs(barInd) * iHs[barInd].diagonal();
			
			//MatrixXF tmp = H.block(fInd,fInd,3,3).inverse() - iH.block(fInd,fInd,3,3);
			//cout << "f"<<i<<" " << tmp.maxCoeff() << "," << tmp.minCoeff() << endl;
			//exit(-1);
		}
		
		barInd++;
		fInd+=3;
	}
	Float invBarz = 1/bs(barInd);
	grads[barInd].head(numOptimTasks) = 2*x.segment(eStart,numOptimTasks);
	grads[barInd](numOptimTasks)      = -2 * x(zStart);
	grad.segment(eStart,numOptimTasks+1) = invBarz*grads[barInd];
	
	
	if (computeHess) {
#ifdef H_USED		
		H.block(eStart,eStart,numOptimTasks+1,numOptimTasks+1) = invBarz*invBarz * grads[barInd]*grads[barInd].transpose();
		H.block(eStart,eStart,numOptimTasks+1,numOptimTasks+1).diagonal()+= invBarz*Hs[barInd].diagonal();
#endif		
		ye = iHs[barInd] * grads[barInd];
		//iH.block(eStart,eStart,numOptimTasks+1,numOptimTasks+1)=bs(barInd) * iHs[barInd] - ye*ye.transpose()/(1+grads[barInd].dot(ye)/bs(barInd));
		iH.block(eStart,eStart,numOptimTasks+1,numOptimTasks+1)=- ye*ye.transpose()/(1+grads[barInd].dot(ye)*invBarz);
		iH.block(eStart,eStart,numOptimTasks+1,numOptimTasks+1).diagonal() += bs(barInd) * iHs[barInd].diagonal(); 
		
		
		
		//MatrixXF tmp =H.block(eStart,eStart,numOptimTasks+1,numOptimTasks+1).inverse() - iH.block(eStart,eStart,numOptimTasks+1,numOptimTasks+1);
		//cout << "e"<<" " << tmp.maxCoeff() << "," << tmp.minCoeff() << endl;
		
		
		//cout << H.block(eStart,eStart,numOptimTasks+1,numOptimTasks+1).inverse() - iH.block(eStart,eStart,numOptimTasks+1,numOptimTasks+1) << endl;
		//exit(-1);
	}
	
	barInd++;
	for (int i=0; i<NJ; i++) {
		Float invBar = 1/bs(barInd);
		grad(i) += invBar;
		H(i,i) = 0;
		if (computeHess) {
			H(i,i)  += invBar*invBar;
		}
		barInd++;
		
		invBar = 1/bs(barInd);
		grad(i) -=  invBar;
		if (computeHess) {
			H(i,i)  +=  invBar*invBar;
		}
		
		if (computeHess) {
			iH(i,i) = 1/H(i,i);
		}
		barInd++;
	}
	//MatrixXF tmp = H.inverse()-H;
	//cout << "tot " << tmp.maxCoeff() << "," << tmp.minCoeff() << endl;
	//cout << "diag " << iH.diagonal() << endl;
	
	//exit(-1);
}

void CustomTaskSpaceController::evalResidual(const VectorXF & x, const VectorXF & v, const MatrixXF & Aeq, const VectorXF & beq, const Float t, bool computeHess, VectorXF & rd, VectorXF & rp, VectorXF & bs, VectorXF& grad, MatrixXF & H, MatrixXF & iH)
{
	evalBarrier(x, computeHess, bs, grad, H,iH);
	rd = rdInit + grad/t + Aeq.transpose()*v;
	rp = Aeq*x-beq;
}



void CustomTaskSpaceController::Optimize() {		
	MatrixXF Aeq(vLen,xLen);
	static bool FirstOptim = true;
	Float tBar = 1;
	Float mu = 150;
	
	if (FirstOptim) {
		cout << "TBar: ";
		cin >> tBar;
		cout << "mu: " ;
		cin >> mu;
	}
	
	static VectorXF f;
	int barEvals = 0;
	Aeq.setZero();
	Aeq.block(0,0,numConstraintTasks,NJ) = Lctau;
	Aeq.block(numConstraintTasks,0,numOptimTasks,NJ) = Lttau;
	Aeq.block(0,NJ,numConstraintTasks,fLen) = Lcf;
	Aeq.block(numConstraintTasks,NJ,numOptimTasks,fLen) = Ltf;
	Aeq.block(numConstraintTasks,NJ+fLen,numOptimTasks,numOptimTasks) = - MatrixXF::Identity(numOptimTasks,numOptimTasks);
	
	
	VectorXF beq(vLen);
	beq.segment(0,numConstraintTasks) = bc;
	beq.segment(numConstraintTasks,numOptimTasks) = bt;
	
	VectorXF x(xLen), v(vLen);
	
	x.setZero();
	if (FirstOptim) {
		for (int i=0; i<NJ; i++) {
			x(i) = 0;
		}
		for (int i=0; i<NP*NS; i++) {
			x(fStart+3*i+0) = .0002;
			x(fStart+3*i+1) = .0004;
			x(fStart+3*i+2) = .001;
		}
		FirstOptim = false;
	}
	else {
		x.segment(0,NJ) = tau;
		x.segment(NJ,3*NP*NS) = f;
	}

	
	for(int i=0;i<numOptimTasks;i++)
	{
		x(eStart+i) = 0;
	}
	//x.segment(eStart,numOptimTasks).setConstant(.1);
	x(zStart) = 10;
	
	Float scaleFact = Aeq.bottomRows(numOptimTasks).maxCoeff()-Aeq.bottomRows(numOptimTasks).minCoeff();
	Aeq.block(numConstraintTasks,NJ+fLen,numOptimTasks,numOptimTasks) = -scaleFact * MatrixXF::Identity(numOptimTasks,numOptimTasks);
	
	
	VectorXF tmp = Aeq*x-beq;
	//cout << "tmp =  " << endl << tmp << endl;;
	x.segment(eStart,numOptimTasks) += tmp.segment(numConstraintTasks,numOptimTasks);
	x(xLen-1) = sqrt(1+pow(x.segment(eStart,numOptimTasks).norm(),2))+1;
	
	//cout << Aeq*x-beq << endl;
	//cout << Aeq.block(numConstraintTasks,eStart,numOptimTasks,numOptimTasks) << endl;
	//exit(-1);
	v.setZero();
	
	
	
	

	//cout << "Aeq = " << endl << Aeq << endl;
	//cout << "beq = " << endl << beq << endl;
	//exit(-1);
	cout << setprecision(5);
	VectorXF rd(xLen), rp(vLen), grad(xLen), rdHat(xLen), rpHat(vLen), gradHat(xLen), bars(barLen), barsHat(barLen);
	MatrixXF H(xLen,xLen), Hhat(xLen,xLen);
	MatrixXF iH(xLen,xLen),iHhat(xLen,xLen);
	
	
	
	JacobiSVD<MatrixXF> AeqSVD(numOptimTasks+numConstraintTasks,xLen);
	AeqSVD.compute(Aeq,ComputeFullV | ComputeFullU);
	MatrixXF V1,U1,V2,U2,E1,iE1, AeqPiv, AeqPivT;
	int AeqRank = AeqSVD.nonzeroSingularValues();
	
	U1 = AeqSVD.matrixU().leftCols(AeqRank);
	V1 = AeqSVD.matrixV().leftCols(AeqRank);
	E1 = AeqSVD.singularValues().head(AeqRank).asDiagonal();
	iE1 = AeqSVD.singularValues().head(AeqRank).asDiagonal().inverse();
	
	U2 = AeqSVD.matrixU().rightCols(numOptimTasks+numConstraintTasks-AeqRank);
	V2 = AeqSVD.matrixV().rightCols(xLen-AeqRank);
	
	AeqPiv  = V1*iE1*U1.transpose();
	AeqPivT = AeqPiv.transpose();
	
	MatrixXF AeqT = Aeq.transpose();
	int AeqNullSpaceDim = xLen-AeqRank;
	
	
	evalResidual(x, v, Aeq, beq, tBar,true, rd, rp, bars, grad, H,iH);
	// Pseudoinverse of Aeq.transpose()
	v = -AeqPivT*rd;
	
	
	
	
	int totalSteps = 0;
	
	
	
	MatrixXF iHAt(xLen,vLen), Khat(vLen,vLen),AiH(vLen,xLen),KhatM(AeqNullSpaceDim,AeqNullSpaceDim);
	VectorXF dv(vLen),dx(xLen);
	dmTimespec t1,t2;
	Float NewtonTime = 0;
	Float BackTrackTime = 0, HessianTime = 0;
	
	
	LDLT<MatrixXF> ldltDecompV(vLen),ldltDecompM(AeqNullSpaceDim);
	
	
	dmGetSysTime(&t1);
	while (tBar < 10000000) {
		int iter = 0;
		rd.setOnes();
		while (rd.norm() > 1e-6) {
			dmTimespec t3,t4;
			dmGetSysTime(&t3);
			barEvals++;
			evalResidual(x, v, Aeq, beq, tBar,true, rd, rp, bars, grad, H,iH);
			
			//cout << "grad = " << grad.transpose() << endl;
			
			//Float b;
			//VectorXF bs(barLen);
			//logBarrier(x, b, bs);
			/*Float bHat;
			//VectorXF bs(barLen);
			
			
			logBarrier(x, b, bs);
			cout << "bs " << endl << bs << endl;
			
			for (int i=0; i<xLen; i++) {
				dx.setZero();
				Float ep = 1e-9;
				dx(i) = ep;
				logBarrier(x+dx, bHat, bs);
				if (isnan(bHat)) {
					cout << "bs = " << bs << endl;
				}
				evalResidual(x+dx, v, Aeq, beq, tBar, rd, rp, bars, gradHat, Hhat,iH);
				
				VectorXF dGrad = (gradHat-grad)/ep - H.col(i);
				
				cout << "i=" << i << " bhat = "<< bHat << " g " << (bHat-b)/ep << "\t" << grad(i) << "\t" << dGrad.norm() << endl;
			}*/
			
			
			//cout << "bars " << endl << bars << endl;
			//cout << "grad " << endl << grad << endl;
			//cout << "H " << endl << H << endl;
			
			//exit(-1);
			H/=tBar;
			iH*=tBar;
			dmGetSysTime(&t4);
			HessianTime+=timeDiff(t3, t4);
			
			/*MatrixXF K(xLen+vLen,xLen+vLen);
			K.setZero();
			K.block(0,0,xLen,xLen) = H;
			K.block(0,xLen,xLen,vLen) = Aeq.transpose();
			K.block(xLen,0,vLen,xLen) = Aeq;
			
			VectorXF rhs(xLen+vLen);
			rhs.head(xLen) = -rd;
			rhs.tail(vLen) = -rp;*/
			
			//cout << "K=" << endl << K << endl;
			//cout << "rd=" << endl << rd<<endl;
			//cout << "rp=" << endl << rp << endl;
			//cout << setprecision(4);
			
			/*ofstream outfile;
			outfile.open("../matlab/data.m");
			
			outfile << setprecision(15);
			//outfile << "K=[" << K << "];" << endl;
			outfile << "H=[" << H << "];" << endl;
			outfile << "iH=[" << iH << "];" << endl;
			outfile << "grad=[" << grad << "];"<<endl;
			outfile << "Aeq=[" << Aeq << "];" << endl;
			outfile << "beq=[" << beq << "];" << endl;
			outfile << "rd=[" << rd << "];" << endl;
			outfile << "rp=[" << rp << "];" << endl;
			outfile << "bs=[" << bs << "];" << endl;
			outfile << "x=["<<x<<"];" <<endl;
			outfile << "t=["<<tBar<<"];"<<endl;
			outfile.close();
			
			exit(-1);*/
			
			//VectorXF dxv = K.fullPivHouseholderQr().solve(rhs);
			//cout << "dxv" << endl;
			//cout << dxv << endl;
			
			//VectorXF NewtonError = K*dxv-rhs;
			//cout << "error = " << NewtonError.norm() << endl;
			//exit(-1);
			
			dmGetSysTime(&t3);
			Float redNorm = norm(rd,rp);
			if (AeqNullSpaceDim < vLen) {
				KhatM = V2.transpose()*H*V2;
				ldltDecompM.compute(KhatM);
				dx = -AeqPiv*rp-V2*ldltDecompM.solve(V2.transpose()*rd);
				dv = AeqPivT*(-rd -H*dx);
			}
			else {
				//iHAt = iH*AeqT;
				AiH = Aeq*iH;
				//Khat = Aeq*iHAt;
				Khat = AiH*AeqT;
				
				ldltDecompV.compute(Khat);
				
				dv = ldltDecompV.solve(rp - AiH*rd);
				//dx = -H.fullPivHouseholderQr().solve(rd+Aeq.transpose()*dv);
				dx = -iH*(rd+AeqT*dv);
			}

			
			dmGetSysTime(&t4);
			//cout << "" << timeDiff(t3, t4) << endl;
			NewtonTime +=timeDiff(t3, t4);
			totalSteps ++;
			
			//cout << "Iter t= " << tBar << " i= " << iter << " rd = " << rd.norm() << " rp = " << rp.norm() << " dxnorm " << dx.norm() << endl;
			
			//dx = dxv.head(xLen);
			//cout << "dx "<< dx.transpose() << endl;
			//cout << "dv "<< dv.transpose() << endl;
			//exit(-1);
			
			/*Float ep = 1;
			evalResidual(x+ep*dx, v+ep*dv, Aeq, beq, tBar, rdHat, rpHat, barsHat, gradHat, Hhat,iHhat);
			
			cout << "rd " << endl << rd << endl;
			cout << "rdhat-" << endl << (rdHat-rd)/ep << endl;
			
			cout << "rp " << endl << rp << endl;
			cout << "rphat-" << endl << (rpHat-rp)/ep << endl;
			
			exit(-1);*/
			
			
			//cout << "Aeq dx" << endl << Aeq*dx << endl;
			
			//exit(-1);
			
	//		cout << "K " << endl << K << endl;
	//		cout << "rd " << endl << rd << endl;
	//		cout << "rp " << endl << rp << endl;
	//		
			dmGetSysTime(&t3);
			Float gamma = 1;
			Float beta = .5;
			Float alpha = .01;
			
	//		cout << "dx " << endl << dx << endl;
	//		cout << "dv " << endl << dv << endl;
			
			barEvals++;
			evalResidual(x+gamma*dx, v+gamma*dv, Aeq, beq, tBar, false, rdHat, rpHat, barsHat, gradHat, Hhat,iHhat);
			Float redNormHat = norm(rdHat,rpHat);
			
	//		cout << "redNormHat " << redNormHat << endl;
	//		cin >> beta;
	//		exit(-1);
			int k=0;
			//cout << "k=" << 0 << " rd = " << rdHat.norm() << " rp = " << rpHat.norm() << " barv " << -barsHat.minCoeff() << endl;
			while ((redNormHat > (1-alpha*gamma)*redNorm) || (barsHat.minCoeff()<0)) {
				gamma *=beta;
				barEvals++;
				evalResidual(x+gamma*dx, v+gamma*dv, Aeq, beq, tBar, false, rdHat, rpHat, barsHat, gradHat, Hhat,iHhat);
				redNormHat = norm(rdHat,rpHat);
				k++;
				//cout << "k=" << k << " rd = " << rdHat.norm() << " rp = " << rpHat.norm() << " barv " << -barsHat.minCoeff() << endl;
			}
			
			dmGetSysTime(&t4);
			BackTrackTime+=timeDiff(t3, t4);
			//cout << "Got " << redNormHat << " req " << (1-alpha*gamma)*redNorm << endl;
			x+=gamma*dx;
			v+=gamma*dv;
			rd = rdHat;
			rp = rpHat;
			//cout << "End t= " << tBar << " i= " <<  iter << " rd = " << rd.norm() << " rp = " << rp.norm() << " k= " << k << endl;
			
			if (k>20) {
				rd.setZero();
			}
			iter++;
			
			//cout << "iter = " << iter << " rd = " << rdHat.norm() << " rp = " << rpHat.norm() << endl;
		}
		//cout << "Complete i= " << iter << " rd = " << rd.norm() << " rp = " << rp.norm() << endl;
		tBar *= mu;
		//cout << "tBar = " << tBar << endl;
	}
	dmGetSysTime(&t2);
	/*cout << "ht=" << HessianTime << endl;
	cout << "nt=" << NewtonTime << endl;
	cout << "bt=" << BackTrackTime << endl;
	cout << "t= " << timeDiff(t1, t2) << endl;
	cout << "x= " << x.transpose() << endl;
	cout << "v= " << v.transpose() << endl;
	cout << "z= " << x.tail(1) << endl;
	cout << "cons " << numConstraintTasks << endl;
	cout << "optims " << numOptimTasks << endl;
	cout << "Ldim vs NSdim " << vLen << " , " << AeqNullSpaceDim << endl;
	cout << "Total Steps = " << totalSteps << endl;
	cout << "BarEvals = " << barEvals << endl;
	
	cout << endl << endl << endl << endl;*/
	//exit(-1);
	
	
	tau = x.segment(0,NJ);
	qdd = VectorXF::Zero(NJ+6);
	fs = VectorXF::Zero(6*NS);
	lambda = VectorXF::Zero(1);
		
	f = x.segment(NJ,3*NS*NP);
	
	VectorXF TaskErrorSub = x.segment(eStart,numOptimTasks)*scaleFact;
	
	int k = 0;
	TaskError.resize(TaskWeight.size());
	for (int i=0; i<TaskWeight.size(); i++) {
		if (taskOptimActive(i) > 0) {
			TaskError(i)= TaskErrorSub(k++)/TaskWeight(i);
		}
		else {
			TaskError(i) = 0;
		}
	}
	
	for (int i=0; i<NS; i++) {
		for (int j=0; j<NP ; j++) {
			Vector3F floc = f.segment(3*(NP*i+j),3);
			floc(2)/=MU;
			fs.segment(6*i,6) += PointForceXforms[i][j].rightCols(3)*floc;
		}
	}
	
	
	
	//exit(-1)
}