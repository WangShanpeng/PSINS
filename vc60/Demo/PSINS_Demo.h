/* PSINS_Demo c++ hearder file KFApp.h */
/*
	By     : Yan Gongmin @ NWPU
	Date   : 2023-04-25
	From   : College of Automation, 
	         Northwestern Polytechnical University, 
			 Xi'an 710072, China
*/

#ifndef _PSINS_Demo_H
#define _PSINS_Demo_H

#include "..\PSINSCore\PSINS.h"

#define FRQ	FRQ200
#define TS	(1.0/FRQ)

#define psinsdemo(demoNO)  \
	if(demoNO>=0) \
	{ \
		switch(demoNO)\
		{\
		case -1: /*main-CKFApp()*/break;\
		case  0: Demo_User(); break;\
		case  1: Demo_CIIRV3(); break;\
		case  2: Demo_CMaxMin(); break;\
		case  3: Demo_CVAR(); break;\
		case  4: Demo_CVARn(); break;\
		case  5: Demo_CRAvar(); break;\
		case  6: Demo_CSINS_static(); break;\
		case  7: Demo_CSINS_Error(); break;\
		case  8: Demo_CAlignsv(); break;\
		case  9: Demo_CAligntf(); break;\
		case 10: Demo_CAlign_CSINS(); break;\
		case 11: Demo_CSINSGNSS(); break;\
		case 12: Demo_CSINSGNSSDR(); break;\
		case 13: Demo_SINSOD(); break;\
		case 14: Demo_POS618(); break;\
		case 15: Demo_CNS_PrecNut(); break;\
		case 16: Demo_SINSCNS(); break;\
		case 17: Demo_IMUTempCompensate(); \
		case 18: Demo_CVCFileFind(); break;\
		case 19: Demo_DSP_main(); break; \
		case 20: Demo_CONSOLE_UART(); break; \
		case 21: Demo_COMMON_UART(); break; \
		case 22: Demo_Cfg(); break; \
		case 23: Demo_CPolyfit(); break; \
		case 24: Demo_CAligni0(); break; \
		case 25: Demo_SysClbt(); break; \
		case 26: Demo_GKP(); break; \
		case 27: Demo_CIMUInc(); break; \
		case 28: Demo_Extern_C_example(); break; \
		case 29: Demo_Extract_Txt_File(); break; \
		case 30: Demo_operator_pointer_run_time(); break; \
		} \
		exit(0); \
	}
void Demo_User(void);
void Demo_CIIRV3(void);
void Demo_CMaxMin(void);
void Demo_CVAR(void);
void Demo_CVARn(void);
void Demo_CRAvar(void);
void Demo_CSINS_static(void);
void Demo_CSINS_Error(void);
void Demo_CAlignsv(void);
void Demo_CAligntf(void);
void Demo_CAlign_CSINS(void);
void Demo_CSINSGNSS(void);
void Demo_CSINSGNSSDR(void);
void Demo_SINSOD(void);
void Demo_POS618(void);
void Demo_CNS_PrecNut(void);
void Demo_SINSCNS(void);
void Demo_IMUTempCompensate(void);
void Demo_CVCFileFind(void);
void Demo_DSP_main(void);
void Demo_CONSOLE_UART(void);
void Demo_COMMON_UART(void);
void Demo_Cfg(void);
void Demo_CPolyfit(void);
void Demo_CAligni0(void);
void Demo_SysClbt(void);
void Demo_GKP(void);
void Demo_CIMUInc(void);
void Demo_Extern_C_example(void);
void Demo_Extract_Txt_File(void);
void Demo_operator_pointer_run_time(void);

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

class CFileIMU7:public CFileRdWt  // read the 7-column IMU bin-file
{
public:
	CVect3 *pwm, *pvm;
	double t0, ts, t;
	CFileIMU7(const char *fname0):CFileRdWt(fname0, -7) {
		load(1);	t0 = buff[6];
		load(1);	ts = buff[6]-t0;
		fseek(rwf, 0L, SEEK_SET);
		pwm=(CVect3*)&buff[0], pvm=(CVect3*)&buff[3];
		t = t0;
	};
	int load(int lines=1) {
		if(!CFileRdWt::load(lines)) return 0;
		t = buff[6];
		return 1;
	};
	void sumIMU(int len, CVect3 &wmm, CVect3 &vmm) {
		long pos=ftell(rwf);
		wmm = vmm = O31;
		for(int i=0; i<len; i++) {
			load(1);
			wmm += *pwm;  vmm += *pvm;
		}
		fseek(rwf, pos, SEEK_SET);
		load(0);
	}
};

class CFileIMUClbt:public CFileRdWt
{
public:
	CIMU *pimu;
	double *pt, *pGyro, *pAcc;
	CFileIMUClbt(const char *fname0, CIMU &imu0):CFileRdWt(fname0, -7) {
		pimu = &imu0;
		pGyro = &buff[0], pAcc = &buff[3];  pt=&buff[6];
		load(1);
	};
	int load(int lines=1) {
		if(!CFileRdWt::load(lines)) return 0;
		return 1;
	};
	CVect3 aligni0(int len, CVect3 &pos0) {
		long pos=ftell(rwf);  int line0=linek;
		CAligni0fit afit(pos0);  // afit.imu=*pimu;  afit.imu.tk=0.0;
		for(int a=0; a<len*FRQ; a++)  {
			load(1);
			pimu->Update((CVect3*)pGyro, (CVect3*)pAcc, 1, TS);
			afit.Update(&pimu->wmm, &pimu->vmm, 1, TS); 
		}
		fseek(rwf, pos, SEEK_SET);  linek=line0;
		CVect3 a1=q2att(afit.qnb)/glv.deg, a2=q2att(afit.qnbsb)/glv.deg;
		psinslog<<"\n Align att (deg): "<<a1<<a2<<"\n";
		return q2att(afit.qnb);
	}
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
		fseek(rwf, sizeof(DataSensor49), SEEK_CUR);
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

typedef struct {
	double t;  //1
	CVect3 vm, wm;  //7
	double temp, dS;  //8
	CVect3 gpos;  // 12
	double na, gvalid;  // 14
} DataSensor14;

class CFileRdSr1:public CFileRdWt  // read sensor file
{
public:
	DataSensor14 *pDS, DS0;
	CVect3 att0;
	CFileRdSr1(const char *fname0):CFileRdWt(fname0, -dbsize(DataSensor14)) {
		pDS = (DataSensor14*)buff;
		load(0);  DS0 = *pDS;       // just get the first record line
	};
	int load(int lines=1) {
		if(!CFileRdWt::load(lines)) return 0;
		pDS->dS *= 1.9606/500;  swapt(pDS->gpos.i, pDS->gpos.j, double);
		return 1;
	};
	CVect3 align(double T2=300, double t0=-1.0) {
		if(t0>0.0) {
			fseek(rwf, 0, SEEK_SET);
			load((int)(t0/TS10));
		}
		CAlignsv aln(pDS->gpos, TS10, T2);
		while(1) {
			if(!load(1) || aln.tk>T2) break;
			aln.Update(&pDS->wm, &pDS->vm);
		}
		return (att0 = q2att(aln.qnb));
	};
};

extern char psinsUartSetting[];

#endif  // PSINS_IO_FILE

#endif  // _PSINS_Demo_H

