/* KFApp c++ hearder file KFApp.h */
/*
	By     : Yan Gongmin @ NWPU
	Date   : 2020-12-09
	From   : College of Automation, 
	         Northwestern Polytechnical University, 
			 Xi'an 710072, China
*/

#ifndef _KFAPP_H
#define _KFAPP_H

#include "PSINS.h"

#define FRQ	FRQ200
#define TS	(1.0/FRQ)

class CKFApp:public CSINSGNSS
{
public:

	CKFApp(double ts);
	virtual void Init(const CSINS &sins0, int grade=-1);
};

typedef struct {
	CVect3 wm, vm, att;
	double t;
	CVect3 gpsvn, gpspos;
} DataSensor;

#ifdef PSINS_IO_FILE
class CFileRdSr:public CFileRdWt  // read sensor record file
{
public:
	DataSensor *pDS, DS0;
	CFileRdSr(const char *fname0):CFileRdWt(fname0, -dbsize(DataSensor)) {
		pDS = (DataSensor*)buff;
//		waitfor((&DS0.gpsvn.i-&DS0.wm.i)/sizeof(double));
		load(1);             // just get the first record line
		DS0 = *pDS;
		bwseek(1);
	};
	int load(int lines=1) {
		if(!CFileRdWt::load(lines)) return 0;
		return 1;
	};
};
#endif

#endif

