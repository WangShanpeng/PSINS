#include "PSINS_Demo.h"

#define PSINSDemo
#ifdef  PSINSDemo

void Demo_User(void)
{
	float ft[100];  double t=-1110.23;
	CVect3 wm(1,2,3), vm(4,5,6);
	int n = iga2flt(ft, (double*)&wm, (double*)&vm, NULL, NULL, &t);
	double tt=flt22db(ft[6], ft[7]);
}

void Demo_CIIRV3(void)
{
	// use Matlab/fdatool to design the IIR filter coefficients
	double Num[] = {0.004824343357716,   0.019297373430865,   0.028946060146297,   0.019297373430865,   0.004824343357716},
		Den[] = {1.000000000000000,  -2.369513007182038,   2.313988414415880,  -1.054665405878568,   0.187379492368185};
	CFileRdWt::Dir(".\\Data\\");	CFileRdWt res("res.bin");
	CIIRV3 iir(Num, Den, sizeof(Num)/sizeof(double));
	for(int k=1; k<100; k++)
	{
		CVect3 x = randn(O31, One31);
		CVect3 y = iir.Update(x);
		res<<x<<y;
	}
}

void Demo_CMaxMin(void)
{
	CFileRdWt::Dir("..\\Data\\", ".\\Data\\");;	CFileRdWt res("res.bin");
	CMaxMin maxmin(20,10);
	for(int k=1; k<100; k++)
	{
		double x = randn(0.0, 1.0);
		int flag = maxmin.Update((float)x);
		res<<x<<maxmin.maxRes<<maxmin.minRes<<maxmin.maxpreRes<<maxmin.minpreRes;
	}
}

void Demo_CVAR(void)
{
	CFileRdWt::Dir("..\\Data\\", ".\\Data\\");;	CFileRdWt res("res.bin");
	CVAR var(20);
	for(int k=1; k<500; k++)
	{
		double x = randn(0.0, 1.0);
		double y = var.Update(x, true);
		res<<x<<(double)var.mean<<var.var;
	}
}

void Demo_CVARn(void)
{
	CFileRdWt::Dir("..\\Data\\", ".\\Data\\");;	CFileRdWt res("res.bin");
	CVARn var(20,1);
	for(int k=1; k<500; k++)
	{
		double x = randn(0.0, 1.0);
		double y = var.Update(x);
		res<<x<<(double)var.mx[0]<<var.stdx[0];
	}
}

void Demo_CRAvar(void)
{
	CFileRdWt::Dir("..\\Data\\", ".\\Data\\");;	CFileRdWt res("res.bin");
	CRAvar ravr(1);
	ravr.set(3.0, 10.0, 10.0, 0.1);
	for(int k=1; k<400; k++)
	{
		double x = randn(0.0, 1.0);  if(k==200) x=10;
		ravr.Update(x, 1.0);
		res<<x<<sqrt(ravr.R0[0]);
	}
}

void Demo_CSINS_static(void)
{
	CFileRdWt::Dir("..\\Data\\", ".\\Data\\");;	CFileRdWt fins("ins.bin");
	double ts=1.0;
	CVect3 att0=PRY(1,1.01,3), vn0=O31, pos0=LLH(34,0,0);
	CVect3 wm[2], vm[2];
	IMUStatic(wm[0], vm[0], att0, pos0, ts); wm[1]=wm[0]; vm[1]=vm[0]; // static IMU simuation
	CSINS sins(a2qua(att0)+CVect3(0,0,0)*glv.min, O31, pos0);
	CCNS cns; double jd=cns.JD(2021,11,1); cns.GetCie(jd,0); CMat3 NP0=cns.CN*cns.CP;
	for(int i=0; i<1*24*3600; i+=2)
	{
		sins.Update(wm, vm, 2, ts);  sins.vn.k=0; sins.pos.k = pos0.k;
		if(i%360==0) {
/*			double T1 = cns.TT + i/(3600*24*36525.0);
			cns.Precmat(T1), cns.Nutmat(T1);
			CMat3 NP = cns.CN*cns.CP;
			CVect3 rv = q2rv(m2qua(NP0*(~NP)));  NP0=NP;
			wm[1] = wm[0]+rv;*/
			fins<<sins;
		}
		else
			wm[1]=wm[0];
		disp(i,1,3600);
	}
}

void Demo_CSINS_Error(void)
{
	CFileRdWt::Dir(".\\Data\\");	CFileRdWt fins("ins.bin");
	CVect3 att0=PRY(0,0,10), vn0=CVect3(1,10,2), pos0=LLH(34,116,340);
	CSINS sins(att0, vn0, pos0, 0.0);
	CSINSGNSS kf(26,6,TS);
	*kf.pphi=CVect3(1,1,10)*MIN, *kf.pdvn=CVect3(0.1,0.2,0.3), *kf.pdpos=V3dpos(10,10,10);
	*kf.peb=CVect3(1,2,3)*DPH, *kf.pdb=CVect3(100,200,300)*UG, *kf.plvr=CVect3(1,2,3)*0; *kf.pdT=0.0;
	*kf.pdkgz=V3SSP(100,100,100), *kf.pdkaii=V3PPP(100,100,100);
	*kf.pddbz=10000000*UGPHPSH;
	if(kf.nq>26) {
		*kf.pdkg1=V3PSS(100,100,100), *kf.pdkg2 =V3SPS(100,100,100), *kf.pdkg3=V3SSP(100,100,100);
		*kf.pdka1=V3PSS(100,100,100), *kf.pdka23=V3SPP(100,100,100);
	}
	kf.Init(CSINS(a2qua(att0)+*kf.pphi, vn0+*kf.pdvn, pos0+*kf.pdpos, 0.0), 2);
	kf.sins.Kg=I33+(~CMat3(O31,O31,*kf.pdkgz)); kf.sins.eb=-*kf.peb;
	kf.sins.Ka=I33+diag(*kf.pdkaii);			kf.sins.db=-*kf.pdb;
	for(int i=0; i<60*FRQ; i++)	{
		CVect3 wm=CVect3(1, 2, 3)*DPS*TS, vm=CVect3(0.1, 0.1, G0+0.1)*TS;
		kf.sins.db.k -= *kf.pddbz*TS;
		sins.Update(&wm, &vm, 1, TS);
		kf.Update(&wm, &vm, 1, TS);
		if(i%10==0) fins<<sins<<kf.sins<<kf.Xk; 
		disp(i, FRQ, 10);
	}
}

void Demo_Extern_C_example(void)  // in main.c file
{
#ifdef PSINS_EXTERN_C_EXAMPLE
	double att[]={0, 0, 0}, vn[]={0, 0, 0}, pos[]={34*DEG, 116*DEG, 340}, t=0.0;
//	double gyro[]={0, 0, 0, 0, 0, 0}, acc[]={0, 0, 9.8*TS, 0, 0, 9.8*TS};
	float gyro[]={0, 0, 0, 0, 0, 0}, acc[]={0, 0, 9.8*TS, 0, 0, 9.8*TS};
	psinsInit0(att, vn, pos, t);
	for(int i=0; i<60*FRQ/2; i++)
	{
//		psinsUpdate0(gyro, acc, 2, TS);
		psinsUpdatef(gyro, acc, 2, TS);
		psinsOut0(att, vn, pos, &t);
		if(i%FRQ==0) printf("%.8f, %.8f, %.3f, %f\n", pos[0]/DEG, pos[1]/DEG, pos[2], t);
	}
#endif
}

void Demo_CAlignsv(void)
{
#ifdef PSINS_RMEMORY
	CFileRdWt::Dir("..\\Data\\", ".\\Data\\");
	CFileIMU6 fimu("lasergyro.imu"); CFileRdWt faln("aln.bin");
	CAlignsv aln(fimu.pos0, fimu.ts, 600, 200);
	for(int i=0; i<600*100; i++)
	{
		if(!fimu.load(1)) break;
		aln.Update(fimu.pwm, fimu.pvm);
		faln<<q2att(aln.qnb)<<aln.tk;
		disp(i, 100, 100);
	}
#endif
}

void Demo_CAligntf(void)
{
	CFileRdWt::Dir("..\\Data\\", ".\\Data\\");
	CFileRdWt fimuavp("transtrj.bin",-16), faln("aln.bin"), fkf("kf.bin");
	struct PS { CVect3 wm, vm, att, vn, pos; double t;};
	PS *pps = (PS*)fimuavp.buff;
	fimuavp.load(1*100);
	CAligntf alntf(CSINS(pps->att+CVect3(1,1,5)*glv.deg, pps->vn, pps->pos, pps->t), TS10);
	for(int i=0; i<600*100; i++)
	{
		if(!fimuavp.load(1)) break;
		alntf.Update(&pps->wm, &pps->vm, 1, TS10);
		if(i%10==0)	{
			alntf.SetMeasVnAtt(pps->vn, pps->att);
		}
		faln<<alntf.sins<<pps->att<<pps->vn<<pps->pos;
		fkf<<alntf;
		disp(i, 100, 10);
	}
}

void Demo_CAlign_CSINS(void)
{
	CFileRdWt::Dir("..\\Data\\", ".\\Data\\");
	CFileIMU6 fimu("lasergyro.imu"); CFileRdWt faln("aln.bin"), fins("ins.bin");
//	CAligni0 aln(pos0);
	CAlignkf aln(CSINS(fimu.att0,fimu.vn0,fimu.pos0),fimu.ts);
	CSINS sins;
	int alnOK=0;
	for(int i=0; i<2000*100; i++)
	{
		if(!fimu.load(1)) break;
		if(i>600*100)
		{
			if(!alnOK) {
				alnOK=1;
				sins.Init(aln.qnb, O31, fimu.pos0, aln.kftk);
			}
			else {
				sins.Update(fimu.pwm, fimu.pvm, 1, fimu.ts);
				fins<<sins;
			}
		}
		else {
			aln.Update(fimu.pwm, fimu.pvm, 1, fimu.ts);
			faln<<q2att(aln.qnb)<<aln.kftk;
		}
		disp(i, 100, 100);
	}
}

void Demo_CSINSGNSS(void)
{
#ifdef PSINS_RMEMORY
	CFileRdWt::Dir("D:\\psins210406\\vc60\\Data\\");
	CFileIMUGNSS fimu("imugps.bin"); CFileRdWt fins("ins.bin"), fkf("kf.bin");
	CAlignsv aln(fimu.pos0, fimu.ts, 50);
	for(double t=0.0; t<50.0; t+=fimu.ts) {
		fimu.load(1); aln.Update(fimu.pwm, fimu.pvm);
	}
	CSINSGNSS kf(19, 6, fimu.ts);
	kf.Init(CSINS(aln.qnb,fimu.vn0,fimu.pos0,fimu.t),0);
	kf.Pmin.Set2(fPHI(0.1,1.0),  fXXX(0.01),  fdPOS(0.1),  fDPH3(0.01),  fUG3(10.0),
			fXXX(0.001), 0.0001, fdKG9(1.0,1.0), fdKA6(1.0,1.0));
	kf.Qt.Set2(fDPSH3(0.01),  fUGPSHZ3(10.0),  fOOO,  fOO6,
			fOOO, 0.0,  fOO9,  fOO6);
	for(int i=0; i<50000*100; i++)
	{
		if(!fimu.load(1)) break;
		kf.Update(fimu.pwm, fimu.pvm, 1, fimu.ts);
		if(fimu.ppGNSS->i>0.1)
			kf.SetMeasGNSS(*fimu.ppGNSS, *fimu.pvGNSS);
		if(i%20==0 || fimu.ppGNSS->i>0.1)
			fins<<kf.sins<<*fimu.pvGNSS<<*fimu.ppGNSS;
		if(i%20==0) fkf<<kf;
		disp(i, 100, 100);
	}
#endif
}

void Demo_CSINSGNSSDR(void)
{
	CFileRdWt::Dir("D:\\ygm2020\\精准\\1229\\");
	CFileRdSr49 fimu("sd5_0122_100hz_x.bin"); CFileRdWt fins("ins.bin"), fkf("kf.bin");
	DataSensor49 *pDS = (DataSensor49*)fimu.buff;

	fimu.waitforgps();
	CSINSGNSSDR kf(TS10);
	kf.Init(CSINS(CVect3(0,0,pDS->gpsyaw),pDS->vngps,pDS->posgps,fimu.t),0);
	for(int i=0; i<5000*100; i++)
	{
		if(!fimu.load(1)) break;
		kf.Update(&pDS->wm, &pDS->vm, pDS->dS, 1, TS10);
		if(pDS->posgps.i>0.1)
			kf.SetMeasGNSS(pDS->posgps, pDS->vngps);
		if(i%10==0||pDS->posgps.i>0.1) {
			fins<<kf.sins<<kf.posDR<<pDS->vngps<<pDS->posgps;
			fkf<<kf;
		}
		disp(i, 100, 100);
	}
}

void Demo_SINSOD(void)
{
	typedef struct { CVect3 wm, vm; double dS; CVect3 att, vn, pos; double t; } DataSensor17;
	CFileRdWt::Dir("D:\\psins210823\\data\\");
	CFileRdWt fins("ins.bin"), fkf("kf.bin");
	CFileRdWt fimu("imuod.bin",-17);	DataSensor17 *pDS=(DataSensor17*)&fimu.buff[0];
	fimu.load(1);
	CVAutoPOS kf(TS10); 
	kf.Init(CSINS(pDS->att, pDS->vn, pDS->pos, pDS->t), 0); 
	kf.sins.AddErr(0.1*DEG);

	for(int i=0; i<6000*FRQ100; i++)
	{
		if(!fimu.load(1)) break;
		kf.Update(&pDS->wm, &pDS->vm, pDS->dS, 1, TS10, 6);
		if (i%10==0)	fins << kf.sins << kf.ODKappa();
		if (i%50==0)	fkf << kf;
		disp(i, FRQ100, 100);
	}
}

void Demo_POS618(void)
{
#ifdef PSINS_RMEMORY
	CFileRdWt::Dir("D:\\ygm2021\\立得空间\\0901\\");
	CPOS618 pos(0.01, 5.0);
	pos.Load("imugnss.bin", 0, 50000);
	pos.Init(0, CVect3(0,0,126)*DEG);
	pos.Process("posres.bin");
#endif
}

void Demo_CNS_PrecNut(void)
{
	FILE *f = fopen("D:\\psins211118\\cns\\sofapn0.txt", "w");
	CCNS cns;
	for(int i=-1000; i<=1000; i++)
	{
		double jd2000 = i*3.6525;
		double T = jd2000/36525;
		cns.Precmat(T); cns.Nutmat(T);
		CVect3 ap=m2att(~cns.CP),  an=m2att(~cns.CN);
		fprintf(f, "%e, %e, %e, %e, %e, %e, %e, %e, %e, %e \n", 2000+jd2000/365.25, ap.i, ap.j, ap.k, an.i, an.j, an.k);
	}
	fclose(f);
}

void Demo_SINSCNS(void)
{
	CFileRdWt::Dir("D:\\ygm2021\\中科华芯\\空工大惯导\\跑车数据20211109\\"); // test_SINS_CNS_to_from_VC60.m
	CFileRdWt fimu("imucns.bin",-11); CFileRdWt fins("ins.bin"), fkf("kf.bin");
	CVect3 *pwm=(CVect3*)&fimu.buff[0], *pvm=(CVect3*)&fimu.buff[3], *pqis=(CVect3*)&fimu.buff[7]; 
	double *pt=&fimu.buff[6], ts=0.01;
	CSINSGNSSCNS kf(ts);
	fimu.load(0);
	kf.SetCNS(2021,11,22,12*3600, -0.1,37);
	kf.Init(CSINS(CVect3(0.01,0.02,0.3)*glv.deg,O31,CVect3(0.59770629,1.9008322240407,380),0.0),0);
	for(int i=0; i<5000*100; i++)
	{
		if(!fimu.load(1)) break;
		kf.Update(pwm, pvm, 1, ts);  kf.sins.pos.k=380;
		if(norm(*pqis)>1e-6)
			kf.SetMeasCNS(*pqis);
		if(i%20==0 || norm(*pqis)>1e-6)
			fins<<kf.sins;
		if(i%20==0) fkf<<kf;
		disp(i, 100, 100);
	}
}

double Sf[] = {
	1.767413e-08, 1.842071e-08, 1.763055e-08, 5.668308e-04, 5.574454e-04, 5.607297e-04 };
double TempArray[] = {
	9.99707708e-01, -3.00658113e-10, 6.04560342e-08, -1.56141601e-05, -2.30622170e-04,
	1.76927602e-03, 3.23551173e-10, -9.28523666e-09, 1.21582445e-06, 2.16338720e-05,
	6.43281415e-04, 2.92715119e-11, -8.38381188e-09, -1.73867682e-06, -2.11605395e-05,
	-1.45768022e-03, -5.62390127e-10, -3.90730479e-08, -1.43763749e-05, -1.91496013e-04,
	9.99675510e-01, 1.17615885e-09, 2.74483237e-08, -1.24579671e-05, -1.71148154e-04,
	2.93618136e-03, 1.42032241e-10, -2.57794180e-08, -2.86285367e-06, -3.34862675e-05,
	-2.02107109e-03, 5.58685695e-12, 6.43358139e-09, 4.49940489e-06, 5.87631230e-05,
	-3.72078288e-04, -2.02255011e-10, 1.00197093e-08, 1.93910812e-06, 2.30112726e-05,
	1.00026338e+00, -1.14345591e-09, 1.21496866e-07, -1.31384570e-05, -2.12316620e-04,
	-6.78028028e-08, 1.01491003e-12, -6.74926973e-11, -1.02697058e-09, 1.03064499e-08,
	1.05081791e-08, 1.37391821e-12, -1.47832115e-10, 1.48511936e-09, 5.72256599e-08,
	-5.83695814e-09, -1.79640335e-12, 1.45214661e-10, -3.44924971e-09, -8.93884927e-08,
	9.97281552e-01, 8.63777438e-10, -3.95437114e-07, 1.57469958e-05, 2.99057997e-04,
	0.00000000e+00, 0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
	0.00000000e+00, 0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
	-3.58137930e-04, -2.56396191e-10, 1.38168200e-08, -1.75352074e-06, -2.86239503e-05,
	9.98471357e-01, 1.60414396e-09, -4.09125432e-07, 2.33152319e-05, 4.07859738e-04,
	0.00000000e+00, 0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
	-3.76213590e-04, -1.24519076e-10, 9.21735098e-09, 3.99490537e-06, 5.29119510e-05,
	2.89612882e-05, -4.50504673e-11, 4.05149029e-10, 5.19375759e-07, 3.40246289e-06,
	9.98259658e-01, -8.89656689e-10, -3.41939533e-07, 2.77677369e-05, 4.51878054e-04,
	-9.54013695e-03, -1.29469908e-09, 5.39897939e-07, -2.22899011e-05, -4.18074610e-04,
	2.54429456e-03, -1.86111376e-10, 5.69954070e-08, -4.86639295e-05, -6.88170679e-04,
	-4.89241637e-03, -5.05265769e-09, 1.30586733e-06, -8.58605188e-05, -1.45977854e-03,
	-5.51462916e-06, -3.88372048e-11, 4.52766300e-09, 2.17483556e-07, 2.14706417e-06,
	-1.30994613e-06, -2.79390677e-11, 2.87778169e-09, 2.54777093e-07, 2.95594310e-06,
	-1.50250682e-06, -4.80516117e-11, 3.76387452e-09, 2.34180366e-07, 2.51754705e-06,
	6.27176736e-02, -2.70814736e-08, 9.23449453e-07, 1.20894345e-05, -5.37388674e-05,
	2.79295927e-04, 8.22297968e-09, -1.67035800e-07, -2.14258729e-05, -2.17305804e-04,
	-3.00929656e-02, -2.87159036e-08, 1.44735644e-06, 4.87172554e-06, -1.12619490e-04,
	1.66365077e-02, -2.20002420e-08, -1.55511007e-07, 1.95472024e-05, 1.15417259e-04,
	6.22343607e-02, 2.37312779e-09, 3.15304389e-07, -2.54577129e-05, -3.50753395e-04,
	-2.89855952e-02, 1.86693879e-08, -9.53931379e-07, 3.40641532e-05, 7.08533949e-04,
	0.00000000e+00, 0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
	0.00000000e+00, 0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
	0.00000000e+00, 0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
	-2.24996722e-03, 6.05198917e-11, -1.64318477e-08, -1.02532799e-07, -1.51699755e-06 };

void Demo_IMUTempCompensate(void)
{
	CIMU imu;
	imu.SetSf(*(CVect3*)&Sf[0], *(CVect3*)&Sf[3]);
	imu.SetTemp(TempArray, 1);
	imu.Temp = 10;
	CVect3 wm(0.1), vm(0.1);
	for(int k=0; k<40; k++)
		imu.Update(&wm, &vm, 1, 0.1);
}

void Demo_CVCFileFind(void)
{
#ifdef PSINS_VC_AFX_HEADER
	CVCFileFind ffind(".\\PSINSCore\\", "*.cpp");
	while(1)
	{
		char *fname = ffind.FindNextFile();
		if(!fname) break;
		printf("%s\n", fname);
	}
#endif  // PSINS_VC_AFX_HEADER
}

// A main-example for DSP/ARM embedded application
void Demo_DSP_main(void)
{
/*	DSP_init(...);
	sampleIMUOK = alignOK = gnssOK = FALSE;
	CAligni0 align(pos0);
	CKFApp kf(TS);
	while(1) {
		if(!sampleIMUOK) continue;
		sampleIMUOK = FALSE;
		if(!alignOK) {
			align.Update(...);
			if(t>300) {
				alignOK = TRUE;
				kf.Init(...);
			}
		}
		else {
			kf.Update(...);  // virtual kf.RTOutput(...);
			if(gnssOK) {
				kf.SetMeasGNSS(...);
				gnssOK = FALSE;
			}
		}
		OutputResult(...);
		t += TS;
	}*/
}


void Demo_CONSOLE_UART(void)
{
#ifdef PSINS_CONSOLE_UART
	FILE *f;
	if(!(f=fopen(".\\Data\\pbcomsetting.txt","rt"))) {
		printf("未找到配置文件 .\\Data\\pbcomsetting.txt！"); getch(); exit(0);	}
	int comi, baudrate; fscanf(f, "%d %d", &comi, &baudrate);  fclose(f);
	CVect3 wm, vm, eb=CVect3(-4.0,1.3,0.0)*glv.dps, db=O31;
	CMahony mahony(10.0);
	CFileRdWt::Dir(".\\Data\\");  CFileRdWt fmh("mahony.bin");
	CConUart uart;
	uart.Init(comi, baudrate, 35*4, 0xaa55);  // COM setting
	for(int i=0; i<5*3600*100; )
	{
		if(uart.getUart())	{
			uart.dispUart();
			wm = (CVect3(uart.ps->gx, uart.ps->gy, uart.ps->gz)*glv.dps - eb)*TS10;
			vm = (CVect3(uart.ps->ax, uart.ps->ay, uart.ps->az)*glv.g0 - db)*TS10;
			mahony.Update(wm, vm, 0.01); mahony.tk=uart.ps->t;
			if(i++%21==0)	{
				uart.gotorc(20,60);
				CVect3 att=q2att(mahony.qnb);
//				printf("Mahony-Att(deg) : %8.3f %8.3f %8.3f", att.i/DEG, att.j/DEG, CC180C360(att.k)/DEG);
				uart.gotorc(21,60);
//				printf("Mahony-eb(deg/s): %8.3f %8.3f %8.3f", mahony.exyzInt.i/DPS, mahony.exyzInt.j/DPS, mahony.exyzInt.k/DPS);
				if(fmh.rwf) fmh<<mahony;
			}
			if(i%100==0&&fmh.rwf) fflush(fmh.rwf);
		}
	}
#endif // PSINS_CONSOLE_UART
}

void Demo_COMMON_UART(void)
{
#ifdef PSINS_CONSOLE_UART
//#define UART_SEND_TEST
#ifdef UART_SEND_TEST
	// 'u16i16i32f32f32f32u16u16'
	ushort hd=0x55aa;	short c=1234;	int i=45;	float f1=2.0, f2=3.0, f3=4.0;	ushort a=1, cksm=2;
	uchar buf[64];
	CComUart uart;
	uart.Init(12, 460800);
	for(int ii=0; ii<5*3600*100; )
	{
		i++;  f1=randn(0.0);  f2=randn(0.0);  f3=randn(0.0);
		int len=makefrm(buf, &hd,2, &c,2, &i,4, &f1,4, &f2,4, &f3,4, &a,2, NULL);
		*(ushort*)&buf[len] = chksum16((ushort*)&buf[2], (len-2)/2);
		uart.sendUart(buf, len+2, 100);
	}
#else
	FILE *f=fopen("psinsUartSetting.txt","rt");
	if(!f) { f=fopen("psinsUartSetting.txt","wt");	fprintf(f, "%s", psinsUartSetting); }
	fclose(f);
//	CFileRdWt::Dir(".\\Data\\");
	CComUart uart;
	uart.Init("psinsUartSetting.txt");
	for(int ii=0; ii<240*3600*100; )
	{
		if(uart.getUart()) {
			uart.dispUart(1); ii++;
		}
	}
#endif // UART_SEND_TEST
#endif // PSINS_CONSOLE_UART
}


void Demo_Cfg(void)
{
	short s; int i; float f; double d; CVect3 v; CQuat q; CMat3 m;
	WriteCfg("psinscfg.cfg")<<"psinscfg"<<(short)12<<23<<1.23f<<3.4<<CVect3(1,4,5)<<CQuat(1,2,3,4)<<CMat3(20);
	ReadCfg("psinscfg.cfg")>>"psinscfg1">>s>>i>>f>>d>>v>>q>>m;
}

void Demo_CPolyfit(void)
{
	CFileRdWt::Dir("..\\Data\\", ".\\Data\\");	CFileRdWt res("res.bin");
	double a[] = {1, 2, 3, 4};
	int n = sizeof(a)/8;
	CPolyfit pfit;
	pfit.Init(1.0, n);
	for(double t=1; t<100; t+=1.0)
	{
		double Zk=0, tj=1.0;
		for(int j=0; j<n; j++)	{ Zk+=a[j]*tj;  tj *= t; }
		Zk += randn(0.0);
		pfit.Update(Zk);
		res << Zk << t;
	}
	double Zk1 = pfit.eval(t);
}

void Demo_CAligni0(void)
{
	CFileRdWt::Dir("..\\Data\\", ".\\Data\\");
	CFileIMU6 fimu("lasergyro.imu"); CFileRdWt faln("aln.bin");
	CAligni0 aln(fimu.pos0);
	CAligni0fit alnfit(fimu.pos0);
	fimu.load(0);
	for(int i=0; i<1800*100; i++)
	{
		if(!fimu.load(1)) break;
		if(i<100) { fimu.pvm->i-=1.0*fimu.ts; fimu.pvm->j-=1.0*fimu.ts; }  // disturb
		if(i>1799*100) { fimu.pvm->i+=1.0*fimu.ts; fimu.pvm->j+=1.0*fimu.ts; }  // disturb
		aln.Update(fimu.pwm, fimu.pvm, 1, fimu.ts);
		alnfit.Update(fimu.pwm, fimu.pvm, 1, fimu.ts);
		faln<<q2att(aln.qnb)<<q2att(alnfit.qnb)<<alnfit.vib0<<alnfit.vn0<<alnfit.vnt<<alnfit.xyzt<<fimu.t;
		disp(i, 100, 100);
	}
}

void Demo_SysClbt(void)
{
int frq=FRQ100;
	// glvs; [imu, att]=imupos19([[1;-91;-92]*glv.deg; glv.pos0], 0.01, 20, 70); imuplot(imu);
	// binfile32('D:\psins240513\data\imupos19.bin', imuclbt(imu));
	CFileRdWt::Dir("D:\\psins240513\\data\\");
	CFileRdWt fimu("imupos19.bin", -7), fclbt("clbt.bin"), fins("ins.bin");
	CVect3 pos0=LLH(34.034310, 108.775427, 450), *pwm, *pvm;	double *pt;
	deal(fimu.buff, &pwm,0, &pvm,3, &pt,6, NULL);
	CAligni0 aln;
	CSysClbt clbt(pos0, 9.795138, 1);  // 1 for ka2, 0 for kapn
	fimu.savepos();
	clbt.sins.Init(O31, O31, pos0, *pt);  clbt.sins.mvnT=0.2;  clbt.sins.isOpenloop=1;
	psinslog.LogSet(1);
	for(int n=0; n<=3; n++)
	{
		fimu.restorepos();  clbt.sins.imu.Reset();  printf(" --- Round %d ---\n", n+1);
		aln.Init(pos0);
		for(int a=0; a<60*frq; a++) {
			fimu.loadf32(1);
			clbt.sins.imu.Update(pwm, pvm, 1, 1.0/frq);
			aln.Update(&clbt.sins.imu.phim, &clbt.sins.imu.dvbm, 1, 1.0/frq);
		}
		fimu.restorepos();
		clbt.NextIter(aln.qnb0);
		CVect3 att=q2att(aln.qnb0)/DEG; printf("ATT: %.4f, %.4f, %.4f\n", att.i, att.j, att.k);
		for(int i=0; i<2*3600*frq; i++)
		{
			if(!fimu.loadf32(1)) break;
			clbt.Update(pwm, pvm, 1, 1.0/frq);
			if(i%100==0) {
				fclbt<<clbt;
				fins<<clbt.sins;
			}
			disp(i, FRQ100, 100);
		}
		clbt.FeedbackAll();
	}
	clbt.Log();
}

void Demo_GKP(void)
{
	CGKP gkp;
	gkp.Init(glv.Re, glv.f);
	for(double lat=60; lat<89; lat+=100.0) {
		for(double lon=0; lon<6; lon+=0.1) {
			CVect3 xyz = gkp.GKP(CVect3(lat*DEG, lon*DEG, 100.0));
			CVect3 pos = gkp.IGKP(xyz);
			printf("%.8f,%.8f \t%.6f %.6f\n", lat, lon, (pos.i-lat*DEG)*RE, (pos.j-lon*DEG)*RE);
		}
	}
}

void Demo_CIMUInc(void)
{
	CVect3 wm(-0.1*SEC,1*SEC,0.1*SEC), vm(-0.0001,0.001,0.0001);
	CIMUInc imu;
	imu.fTotalGx=-(2147483648.0-15), imu.fTotalGy=imu.fTotalGz=2147483648.0-15;
	imu.fTotalAx=-(2147483648.0-15), imu.fTotalAy=imu.fTotalAz=2147483648.0-15;
	for(int i=0; i<20; i++) {
		imu.Update(wm, vm);
		printf("%d,%d,%d\t%d,%d,%d \t%d,%d,%d\t%d,%d,%d\n",
			imu.diGx, imu.diGy, imu.diGz, imu.diAx, imu.diAy, imu.diAz,
			imu.iTotalGx, imu.iTotalGy, imu.iTotalGz, imu.iTotalAx, imu.iTotalAy, imu.iTotalAz);
	}
}

void Demo_Extract_Txt_File(void)
{
	char str[1024], hdr[]="GR,";  int len=strlen(hdr), n=0;
	FILE *fIn =fopen("E:\\ygm2023\\FAST\\202306280731db2.log","rt"), 
		 *fOut=fopen("E:\\ygm2023\\FAST\\gps0628.log","wt");
	for(int i=0; i<1000000000; i++) {
		fgets(str, 1024, fIn);
		if(feof(fIn)) break;
		if(chkhdr(str, hdr)) {
			fprintf(fOut, "%s", &str[len]);
			disp(n, 10, 100);  n++;
		}
	}
	fclose(fIn); fclose(fOut);
}

void Demo_operator_pointer_run_time(void)
{
	CVect3 v1=randn(O31)*0.001, v2=randn(O31), v, vv1,vv2;
	CMat3 m1=randn(O33), m2=randn(O33), m;
	CQuat q1(1,0,0,0), q;
	int len=10000*1000, i;
	for(i=0; i<len; i++) {
//		m = m1*m2;
//		v = m1*v1;
//		v = v1*v2;
//		q = q1-v1;
//		v = m1*v1+v1*2.0;
		vv1 = v1*2.0+v2*3.0;
//		v = m2att(m1);
//		v = v1+v2*3.0;
//		q1 -= v1;
//		v1 -= v1;
	}
	glv.toc(1);
	for(i=0; i<len; i++) {
//		mul(m, m1, m2);
//		mul(v, m1, v1);
//		cros(v, v1, v2);
//		qdelphi(q1, v1);
//		AXbt(v, m1, v1, v1, 2.0);
//		v1.i = asin(v1.k*v1.k/1000.0);
		VADDff(vv2, v1, 2.0, v2, 3.0);
//		m2att(v, m1);
//		VADDf(v, v1, v2, 3.0);
//		qdelphi(q1, v1);
//		sub(v1,v1,v1);
//		VSUB(v,v1,v2);
	}
}

char psinsUartSetting[] = 
"11  460800  0xaa55  0    一个psins串口设置的例子\n"
"u16  *1     帧头%10.1f\n"
"i16  *1.23  aaa-%10.2f\n"
"i32  *1.0   bbb-%10.2f\n"
"f32  *1.23  陀螺x(°/s)%10.3f\n"
"f32  /1.23  陀螺y(°/s)%10.3f\n"
"f32  /1.23  陀螺z(°/s)%10.3f\n"
"n16  *1.0   保留\n"
"u16  *1.0   校验%10.2f\n"
"#\n"
"\n"
"----------------------------------- 配置规范 ------------------------------------------\n"
"示例：\n"
"COMn  Baudrate  Header  0/1  comments  ---第1行：串口号 波特率 帧头 大/小端模式  注释\n"
"u16       *1.0   frame_header%10.1f\n  ---第2行：显示帧头信息\n"
"datatype  scale  printf_format\n       ---第n行：数据类型  比例系数  显示信息\n"
"datatype  scale  printf_format\n"
"...\n"
"#                                      ---最后行：结束符#\n"
"\n"
"psinsUartSetting.txt串口配置文件编写规范及注意事项：\n"
"（1）本软件仅应用于固定帧长格式的二进制串口数据接收，\n"
"     必须严格按本配置规范编写帧格式，才能正确接收、存储及在控制台窗口输出显示；\n"
"（2）可先用‘串口助手’之类软件查看串口配置及接收是否正常；\n"
"（3）第1行第1列数据为串口号，第2列数据为波特率，默认数据位8、停止位1.5、无奇偶校验；\n"
"（4）第1行第3列数据为帧头，帧头必须为两字节，以0x打头；\n"
"（5）第1行第4列数据为端模式，1为小端模式，0为大端模式；\n"
"（6）第1行第5列信息为注释，不能缺省，随便写点什么都行（不留空格），但也不要太长；\n"
"（7）第n>1行首字符为数类型：i-整型，u-无符号整型，f-浮点型，n-无类型（用于跳过标记）\n"
"     之后跟8/16/24/32/64分别为数据比特数（除8即为字节数），nxxx意为跳过xxx比特不显示；\n"
"（8）第n>1行第2列为比例系数，数据前带*号为乘、带/号为除，若为*1也不能省略；\n"
"（9）第n>1行第3列为窗口显示信息，所有数据均以浮点方式显示，若不写格式符%号则不显示；\n"
"     第3列显示信息应为完整字符串，中间不得留空格；\n"
"（10）最后行以首字符#结束帧格式配置；\n"
"（11）比例系数和显示均不影响数据存储，直接将接收到整帧信息按二进制存储至文件。\n"
"\n" ;

#endif  // PSINSDemo
