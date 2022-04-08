#include "KFApp.h"

/***************************  class CKFApp  *********************************/
CKFApp::CKFApp(double ts):CSINSGNSS(19, 6, ts)
{
//state: 0-2 phi; 3-5 dvn; 6-8 dpos; 9-11 eb; 12-14 db; 15-17 lever; 18 dt
//meas:  0-2 dvn; 3-5 dpos
}

void CKFApp::Init(const CSINS &sins0, int grade)
{
	CSINSGNSS::Init(sins0);
	Pmax.Set2(fPHI(600,600),  fXXX(500),  fdPOS(1e6),  fDPH3(5000),  fMG3(10), fXXX(10),  0.1);
	Pmin.Set2(fPHI(0.1,1.0),  fXXX(0.001),  fdPOS(0.1),  fDPH3(0.1),  fUG3(10), fXXX(0.01),  0.0001);
	Pk.SetDiag2(fPHI(60,600),  fXXX(1.0),  fdPOS(10.0),  fDPH3(100),  fMG3(3.0), fXXX(1.0),  0.01);
	Qt.Set2(fDPSH3(0.1),  fUGPSHZ3(1.0),  fOOO,  fOO6,	fOOO, 0.0);
	Rt.Set2(fXXZ(0.5,1.0),   fdLLH(10.0,30.0));
	SetRmmbt(0.1, 10, 0.6);
	FBTau.Set(fXX9(0.1),  fXX6(1.0),  fINF3, INF);
}
