#include ".\PSINSCore\kfapp.h"
#include ".\Demo\PSINS_Demo.h"

void main(void)
{
	CFileRdWt::Dir("E:\\ygm2024\\FAST\\1FAST现场标定0826\\");
	CFileRdWt fwvm("wvm.bin");
	CIMUInc inc;
	double ts=0.01;
	for(int ii=0; ii<100*100; ii++)
	{
		double t = ii*ts;
		CVect3 wm(0.1*DPS*ts, 1*DPS*ts, 10*DPS*ts), vm(0.1*ts, 1*ts, 10*ts); 
		wm = wm*t/1.0;  vm = vm*t/1.0;
		inc.Update(wm, vm);
		fwvm<<CVect3(inc.diGx, inc.diGy, inc.diGz)*inc.gScale<<CVect3(inc.diAx, inc.diAy, inc.diAz)*inc.aScale<<t;
	}
	return;

//	ClassSizeDisp();
	psinsdemo(25);

	CFileRdWt::Dir("D:\\ygm2020\\PSINS网站\\惯导数据\\", "D:\\psins210207\\VC60\\Data\\");
	CFileRdWt fins("ins.bin"), fkf("kf.bin");
	CFileRdSr fimu("mimuattgps.bin");  // download from: http://www.psins.org.cn/newsinfo/958984.html
	DataSensor *pDS=(DataSensor*)fimu.buff, *pDS0=&fimu.DS0;

	CKFApp kf(TS);
	kf.Init(CSINS(pDS0->att, pDS0->gpsvn, pDS0->gpspos, pDS0->t));

	for(int i=0; i<5000*FRQ; i++)
	{
		if(!fimu.load(1)) break;
		if(pDS->gpspos.i>0.1 && !hit3(pDS->t,500,600,900,1000,2000,2100))
		{
			kf.SetMeasGNSS(pDS->gpspos, pDS->gpsvn);
		}
		kf.Update(&pDS->wm, &pDS->vm, 1, TS);

		if(i%5==0||pDS->gpspos.i>0.1)
		{
			fins << kf.sins << pDS->att;
			fkf << kf;
		}

		disp(i, FRQ, 100);
	}
}
