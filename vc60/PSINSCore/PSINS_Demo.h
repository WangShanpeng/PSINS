/* PSINS_Demo c++ hearder file KFApp.h */
/*
	By     : Yan Gongmin @ NWPU
	Date   : 2021-01-21
	From   : College of Automation, 
	         Northwestern Polytechnical University, 
			 Xi'an 710072, China
*/

#ifndef _PSINS_Demo_H
#define _PSINS_Demo_H

#include "PSINS.h"

#define PSINSDemo 14

#define psinsdemo()  \
	switch(PSINSDemo)\
	{\
	case -1: /*main-CKFApp()*/break;\
	case 0: Demo_User(); break;\
	case 1: Demo_CIIRV3(); break;\
	case 2: Demo_CMaxMin(); break;\
	case 3: Demo_CVAR(); break;\
	case 4: Demo_CVARn(); break;\
	case 5: Demo_CRAvar(); break;\
	case 6: Demo_CSINS_static(); break;\
	case 7: Demo_CAlignsv(); break;\
	case 8: Demo_CAligntf(); break;\
	case 9: Demo_CAlign_CSINS(); break;\
	case 10: Demo_CSINSGNSS(); break;\
	case 11: Demo_CSINSGNSSDR(); break;\
	case 12: Demo_CVCFileFind(); break;\
	case 13: Demo_DSP_main(); break; \
	case 14: Demo_CONSOLE_UART(); break; \
	}
void Demo_User(void);
void Demo_CIIRV3(void);
void Demo_CMaxMin(void);
void Demo_CVAR(void);
void Demo_CVARn(void);
void Demo_CRAvar(void);
void Demo_CSINS_static(void);
void Demo_CAlignsv(void);
void Demo_CAligntf(void);
void Demo_CAlign_CSINS(void);
void Demo_CSINSGNSS(void);
void Demo_CSINSGNSSDR(void);
void Demo_CVCFileFind(void);
void Demo_DSP_main(void);
void Demo_CONSOLE_UART(void);

#ifdef PSINS_IO_FILE

class CFileIMU6:public CFileRdWt  // read the 6-column IMU file created by 'imufile.m'
{
public:
	CVect3 *pwm, *pvm, att0, vn0, pos0, gf, af;
	double t0, ts, g0, t;
	CFileIMU6(const char *fname0):CFileRdWt(fname0, 6) {
		CFileRdWt::load(1);	att0=*(CVect3*)&buff[0]*glv.deg, vn0=*(CVect3*)&buff[3];
		CFileRdWt::load(1);	pos0=LLH(buff[0],buff[1],buff[2]);	t0=buff[3], ts=buff[4]/1000.0, g0=buff[5];
		CFileRdWt::load(1);	gf=*(CVect3*)&buff[0]*glv.sec, af=*(CVect3*)&buff[3]*g0*1.0e-6;
		pwm=(CVect3*)&buff[0], pvm=(CVect3*)&buff[3];
		t = t0;
	};
	int load(int lines=1, BOOL txtDelComma=1) {
		if(!CFileRdWt::load(lines)) return 0;
		buff[0]*=gf.i, buff[1]*=gf.j, buff[2]*=gf.k;
		buff[3]*=af.i, buff[4]*=af.j, buff[5]*=af.k;
		t += ts;
		return 1;
	};
};

class CFileIMUGNSS:public CFileRdWt  // read the 13-column IMU/GPS bin file created by 'binfile.m'
{
public:
	CVect3 *pwm, *pvm, *pvGNSS, *ppGNSS, vn0, pos0;
	double t0, ts, t;
	CFileIMUGNSS(const char *fname0):CFileRdWt(fname0, -13) {
		pwm=(CVect3*)&buff[0], pvm=(CVect3*)&buff[3], pvGNSS=(CVect3*)&buff[6], ppGNSS=(CVect3*)&buff[9];
		CFileRdWt::load(1);	vn0=*pvGNSS, pos0=*ppGNSS, t0=buff[12];
		CFileRdWt::load(1);	t=buff[12]; ts=t-t0;
		if(IsZero(vn0)) {
			while(IsZero(*pvGNSS)) if(!load(1)) break;  // wait for first GNSS
			vn0=*pvGNSS, pos0=*ppGNSS;
		}
	};
	int load(int lines=1) {
		if(!CFileRdWt::load(lines)) return 0;
		t += ts;
		return 1;
	};
};

/* an example for user sensor file read, always coded in kfapp.cpp
typedef struct {
	double t;
	CVect3 wm, vm;
	double dS;  //7
	CVect3 posgps; double dop; CVect3 vngps; double yaw; //15
	CVect3 pos610, att610, vn610; //24
} DataSensor;

class CFileRdSr:public CFileRdWt  // CFile Read Sensor
{
public:
	DataSensor *pDS, DS0;
	CFileRdSr(const char *fname0, int columns0=0):CFileRdWt(fname0, columns0) {
		pDS = (DataSensor*)buff;
		load(1);             // get the first record line
		memcpy(&DS0, pDS, sizeof(DataSensor));
		fseek(f, sizeof(DataSensor), SEEK_CUR);
	};
	int load(int lines=1) {
		if(!CFileRdWt::load(lines)) return 0;
		swap(pDS->att610.i, pDS->att610.j, double);
		return 1;
	};
	BOOL hit(double t10, double t11, double t20=INF, double t21=-INF, double t30=INF, double t31=INF) {
		double t = pDS->t - DS0.t;
		if( (t10<t&&t<t11) || (t20<t&&t<t21) || (t30<t&&t<t31) ) return TRUE;
		return FALSE;
	};
};
*/

typedef struct {
	CVect3 wm, vm; //6
	double na1[9]; //15
	CVect3 vngps, posgps; //21
	double na2[1];
	double gpsyaw; //23
	CVect3 att, vn, pos; //32
	double na3[7]; //39
	double dS;  //40
	double na4[9];
} DataSensor49;

class CFileRdSr49:public CFileRdWt  // CFile Read Sensor
{
public:
	double t0, t;
	DataSensor49 *pDS, DS0;
	CFileRdSr49(const char *fname0):CFileRdWt(fname0, -dbsize(DataSensor49)) {
		pDS = (DataSensor49*)buff;
		t0 = t = 0;
		load(1);             // get the first record line
		memcpy(&DS0, pDS, sizeof(DataSensor49));
		fseek(f, sizeof(DataSensor49), SEEK_CUR);
	};
	int load(int lines=1) {
		if(!CFileRdWt::load(lines)) return 0;
		pDS->wm *= glv.dps*TS10;  pDS->vm *= glv.g0*TS10;
		pDS->posgps = LLH(pDS->posgps.j, pDS->posgps.i, pDS->posgps.k);  pDS->gpsyaw *= DEG;
		pDS->dS = pDS->dS>0 ? pDS->dS *= 0.1 : 0.0;
		t += TS10;
		return 1;
	};
	int waitforgps(void) {
		while(pDS->posgps.i<0.1) if(!load(1)) return 0;
		return 1;
	}
};

#endif  // PSINS_IO_FILE

#endif  // _PSINS_Demo_H

