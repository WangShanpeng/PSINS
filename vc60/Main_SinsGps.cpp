#include ".\PSINSCore\kfapp.h"
#include ".\Demo\PSINS_Demo.h"

void main(void)
{
//	ClassSizeDisp();
	psinsdemo(25);

	CFileRdWt::Dir("D:\\ygm2020\\PSINSÍøÕ¾\\¹ßµ¼Êý¾Ý\\", "D:\\psins210207\\VC60\\Data\\");
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
