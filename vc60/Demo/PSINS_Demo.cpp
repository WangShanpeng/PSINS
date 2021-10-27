#include "PSINS_Demo.h"

#ifdef  PSINSDemo

void Demo_User(void)
{
	CVect3 att = sv2att(CVect3(2,3,1), 0.0, CVect3(1,2,3))/DEG;
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
		int flag = maxmin.Update(x);
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
	double ts=1, t;
	CVect3 att0=PRY(1,1.01,3), vn0=O31, pos0=LLH(34,0,0);
	CVect3 wm[2], vm[2];
	IMUStatic(wm[0], vm[0], att0, pos0, ts); wm[1]=wm[0]; vm[1]=vm[0]; // static IMU simuation
	CSINS sins(a2qua(att0)+CVect3(0,0,1)*glv.min, O31, pos0);
	for(t=0.0; t<24*3600.0; t+=2*ts)
	{
		sins.Update(wm, vm, 2, ts);  sins.pos.k = pos0.k;
		fins<<sins;
	}
}

void Demo_CAlignsv(void)
{
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
	CFileRdWt::Dir("D:\\ygm2021\\立得空间\\0901\\");
	CPOS618 pos(0.01, 5.0);
	pos.Load("imugnss.bin", 0, 50000);
	pos.Init(0, CVect3(0,0,126)*DEG);
	pos.Process("posres.bin");
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

#ifdef PSINS_CONSOLE_UART

void Demo_CONSOLE_UART(void)
{
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
}

#endif // PSINS_CONSOLE_UART

#endif  // PSINSDemo
