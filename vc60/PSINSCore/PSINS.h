
/* PSINS(Precise Strapdown Inertial Navigation System) C++ algorithm hearder file PSINS.h

Copyright(c) 2015-2023, by YanGongmin, All rights reserved.
Northwestern Polytechnical University, Xi'an, P.R.China.
Date: 17/02/2015, 19/07/2017, 11/12/2018, 27/12/2019, 12/12/2020, 22/11/2021, 17/10/2022, 23/09/2023
      16/06/2024
*/

#ifndef _PSINS_H
#define _PSINS_H

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>

#pragma pack(4)

/************** compiling control !!! ***************/
#define PSINS_MATRIX_MAX_DIM	40
#define PSINS_IO_FILE
//#define PSINS_IO_FILE_FIND
#define PSINS_RMEMORY
//#define PSINS_AHRS_MEMS
//#define PSINS_psinsassert
//#define PSINS_MAT_COUNT
//#define PSINS_STACK
//#define PSINS_CONSOLE_UART
//#define PSINS_VC_AFX_HEADER
//#define PSINS_COMPLEX
//#define PSINS_FAST_CALCULATION
//#define PSINS_EXTERN_C_EXAMPLE
//#define PSINS_FOR_LINUX

#ifdef PSINS_IO_FILE
#ifdef PSINS_FOR_LINUX  // for Linux
	#undef PSINS_IO_FILE_FIND
	#undef PSINS_CONSOLE_UART
	#undef PSINS_VC_AFX_HEADER
	#define kbhit()		NULL
	#define getch()		NULL
	#include <unistd.h>
#else
	#include <conio.h>
	#include <io.h>
#endif
#endif

// type re-define
#ifndef BOOL
typedef int		BOOL;
#endif

#ifndef BYTE
typedef unsigned char BYTE;
#endif

#ifndef uchar
typedef unsigned char uchar;
#endif

#ifndef ushort
typedef unsigned short ushort;
#endif

#ifndef uint
typedef unsigned int uint;
#endif

// constant define
#ifndef TRUE
#define TRUE	1
#define FALSE	0
#endif

#ifndef NULL
#define NULL	((void *)0)
#endif

#ifndef PI
#define PI		3.14159265358979
#endif
#define PI_2	(PI/2.0)
#define PI_4	(PI/4.0)
#define _2PI	(2.0*PI)

#define sqrt2	1.414213562373095	// sqrt(2) ...
#define sqrt3	1.732050807568877
#define sqrt5	2.236067977499790
#define sqrt6	2.449489742783178
#define sqrt7	2.645751311064591
#define sqrt8	2.828427124746190

#define DEG		(PI/180.0)		// arcdeg
#define MIN		(DEG/60.0)		// arcmin
#define SEC		(MIN/60.0)		// arcsec
#define HUR		3600.0			// hur
#define SHUR	60.0			// sqrt(hur)
#define DPS		(DEG/1.0)		// deg/s
#define DPH		(DEG/HUR)		// deg/h
#define DPSH	(DEG/SHUR)		// deg/sqrt(h)
#define G0		9.7803267714
#define MG		(G0/1.0e3)
#define UG		(G0/1.0e6)		// ug
#define UGPH	(UG/HUR)		// ug/h
#define UGPHPSH	(UGPH/SHUR)		// ug/h/sqrt(h)
#define UGPSHZ	(UG/1)			// ug/sqrt(Hz)
#define UGPG2	(UG/G0/G0)		// ug/g^2
#define SECPG	(SEC/G0)		// sec/g
#define PPM		1.0e-6
#define RE		6378137.0
#define FF		(1.0/298.257)
#define WIE		7.2921151467e-5

#ifndef EPS
#define EPS		(2.220446049e-16)
#endif
#ifndef INF
#define INF		(3.402823466e+30)
#endif
#define INFp5	(INF*0.5)
#define fEND	(10.0*INF)

#define FRQ1		1				// sampling frequency (FRQ** Hz)
#define FRQ5		5
#define FRQ10		10
#define FRQ25		25
#define FRQ50		50
#define FRQ100		100
#define FRQ125		125
#define FRQ200		200
#define FRQ250		250
#define FRQ400		400
#define FRQ500		500
#define FRQ800		800
#define FRQ1000		1000
#define TS1			(1.0/FRQ1000)	// sampling interval (TS** ms)
#define TS1p25		(1.0/FRQ800)
#define TS2			(1.0/FRQ500)
#define TS2p5		(1.0/FRQ400)
#define TS4			(1.0/FRQ250)
#define TS5			(1.0/FRQ200)
#define TS8			(1.0/FRQ125)
#define TS10		(1.0/FRQ100)
#define TS20		(1.0/FRQ50)
#define TS40		(1.0/FRQ25)
#define TS100		(1.0/FRQ10)
#define TS200		(1.0/FRQ5)
#define TS1000		(1.0/FRQ1)

// constant define for short in KF P/Q/R setting
#define fXYZU(X,Y,Z,U)	1.0*(X)*(U),1.0*(Y)*(U),1.0*(Z)*(U)
#define fXXZU(X,Z,U)	fXYZU(X,X,Z,U)
#define fXYZ(X,Y,Z)		fXYZU(X,Y,Z,1.0)
#define fXXZ(X,Z)		fXYZ(X,X,Z)
#define fXXX(X)			fXYZ(X,X,X)
#define fXX6(X)			fXXX(X),fXXX(X)
#define fXX9(X)			fXX6(X),fXXX(X)
#define fOOO			fXXX(0.0)
#define fOO6			fXX6(0.0)
#define fOO9			fXX9(0.0)
#define fIII			fXXX(1.0)
#define fII6			fXX6(1.0)
#define fII9			fXX9(1.0)
#define fINF3			fXXX(INF)
#define fINF6			fXX6(INF)
#define fINF9			fXX9(INF)
#define fPHI(EN,U)		fXXZU(EN,U,MIN)
#define fMU(X,Y,Z)		fXYZU(X,Y,Z,MIN)
#define fdVEL(X)		fXXX(X)
#define fdLLH(LL,H)		fXXZ((LL)/RE,(H))
#define fdPOS(LLH)		fdLLH(LLH,LLH)
#define fDEG3(X)		fXXX(X*DEG)
#define fDEGXXZ(X,Z)	fXXZU(X,Z,DEG)
#define fMIN3(X)		fXXX(X*MIN)
#define fMINXXZ(X,Z)	fXXZU(X,Z,MIN)
#define fSEC3(X)		fXXX(X*SEC)
#define fSECXXZ(X,Z)	fXXZU(X,Z,SEC)
#define fDPS3(X)		fXXX(X*DPS)
#define fDPSXXZ(X,Z)	fXXZU(X,Z,DPS)
#define fDPH3(X)		fXXX(X*DPH)
#define fDPHXXZ(X,Z)	fXXZU(X,Z,DPH)
#define fDPSH3(X)		fXXX(X*DPSH)
#define fDPSHXXZ(X,Z)	fXXZU(X,Z,DPSH)
#define fMG3(X)			fXXX(X*MG)
#define fMGXXZ(X,Z)		fXXZU(X,Z,MG)
#define fUG3(X)			fXXX(X*UG)
#define fUGXXZ(X,Z)		fXXZU(X,Z,UG)
#define fUGPSHZ3(X)		fXXX(X*UGPSHZ)
#define fUGPSHZXXZ(X,Z)	fXXZU(X,Z,UGPSHZ)
#define fPPM3(X)		fXXX(X*PPM)
#define fKPP(dpch,dk,dyaw)	fXYZ(dpch*DEG,dk,dyaw*DEG)
#define fdKPSS(p,s1,s2)		(p)*PPM,(s1)*SEC,(s2)*SEC
#define fdKSPS(s1,p,s2)		(s1)*SEC,(p)*PPM,(s2)*SEC
#define fdKSSP(s1,s2,p)		(s1)*SEC,(s2)*SEC,(p)*PPM
#define fdKSPP(s,p1,p2)		(s)*SEC,(p1)*PPM,(p2)*PPM
#define fdKPPP(p1,p2,p3)	(p1)*PPM,(p2)*PPM,(p3)*PPM
#define fdKG1(dkii)			(dkii)*PPM			// dkzz
#define fdKG3(dkii,dkij)	(dkij)*SEC,(dkij)*SEC,(dkii)*PPM		// dkxz,dkyz,dkzz
#define fdKG9(dkii,dkij)	(dkii)*PPM,(dkij)*SEC,(dkij)*SEC,(dkij)*SEC,(dkii)*PPM,(dkij)*SEC,(dkij)*SEC,(dkij)*SEC,(dkii)*PPM
#define fdKA9(dkii,dkij)	(dkii)*PPM,(dkij)*0.0,(dkij)*0.0,(dkij)*SEC,(dkii)*PPM,(dkij)*0.0,(dkij)*SEC,(dkij)*SEC,(dkii)*PPM
#define fdKA6(dkii,dkij)	(dkii)*PPM,(dkij)*SEC,(dkij)*SEC,(dkii)*PPM,(dkij)*SEC,(dkii)*PPM
#define fdKGA15(dkgii,dkgij,dkaii,dkaij)	fdKG9(dkgii,dkgij),fdKA6(dkaii,dkaij)
//#define fdKGA(dkii,x1,x2,x3)				fdKG1(dkii)
//#define fdKGA(dkii,dkij,x2,x3)			fdKG3(dkii,dkij)
//#define fdKGA(dkii,dkij,x2,x3)			fdKG9(dkii,dkij)
#define fdKGA(dkgii,dkgij,dkaii,dkaij)	fdKGA15(dkgii,dkgij,dkaii,dkaij)
#define fdKa23(dka2)	fXXX(dka2*UGPG2)
#define fdKapn3(dkapn)	fXXX(dkapn*PPM)
#define fInLv3(lv)		fXXX(lv*0.01)	// in cm
#define fInLv6(lv)		fInLv3(lv),fInLv3(lv)
#define SECPG3(X)  fXXX(X*SECPG)
#define SECPG9(X)  SECPG3(X),SECPG3(X),SECPG3(X)


#define dbsize(datatype)	( (int)(sizeof(datatype)/sizeof(double)) )
#define Deletep(p)			{ if(p) { delete p; p=NULL; } }

#ifdef PSINS_psinsassert
	BOOL	psinsassert(BOOL b);
#else
	#define psinsassert(b)  {};
#endif

#ifndef mmax
#define mmax(x,y)        ( (x)>=(y)?(x):(y) )
#endif
#ifndef mmin
#define mmin(x,y)        ( (x)<=(y)?(x):(y) )
#endif

#define IIR1(y,x,afa)	( (1.0-(afa))*(y)+(afa)*(x) )		// yk = (1.0-afa)*yk_1 + afa*xk
#define CC180C360(yaw)  ( (yaw)>0.0 ? (_2PI-(yaw)) : -(yaw) )   // counter-clockwise +-180deg -> clockwise 0~360deg for yaw
#define C360CC180(yaw)  ( (yaw)>=PI ? (_2PI-(yaw)) : -(yaw) )   // clockwise 0~360deg -> counter-clockwise +-180deg for yaw
#define LLH(latitude,longitude,height)	CVect3((latitude)*DEG,(longitude)*DEG,height)
#define PRY(pitch,roll,yaw)		CVect3((pitch)*DEG,(roll)*DEG,C360CC180((yaw)*DEG))
#define V3PPP(p1,p2,p3)		CVect3(p1*PPM,p2*PPM,p3*PPM)
#define V3PSS(p1,s2,s3)		CVect3(p1*PPM,s2*SEC,s3*SEC)
#define V3SPS(s1,p2,s3)		CVect3(s1*SEC,p2*PPM,s3*SEC)
#define V3SSP(s1,s2,p3)		CVect3(s1*SEC,s2*SEC,p3*PPM)
#define V3SPP(s1,p2,p3)		CVect3(s1*SEC,p2*PPM,p3*PPM)
#define V3dpos(dlat,dlon,dhgt)	CVect3((dlat)/RE,(dlon)/RE,dhgt)
#define LLHcvt(pllh)	{ double *p=(double*)pllh,tmp=p[0]; p[0]=p[1]*DEG,p[1]=tmp*DEG; }  // lat/lon/hgt convert
#define PRYcvt(ppry)	{ double *p=(double*)ppry,y=p[2]*DEG; p[0]=p[0]*DEG,p[1]=p[1]*DEG,p[2]=C360CC180(y); }  // pch/rll/yaw convert


#ifdef PSINS_STACK
extern int	psinsstack0, psinsstacksize;
#define stackstart()	{ int stack0=0; psinsstack0=(int)&stack0; }
#define stacksize()		{ int stack1=0; psinsstacksize=mmax(psinsstacksize,psinsstack0-(int)&stack1); }
#else
#define stackstart()	NULL
#define stacksize()		NULL
#endif

#define disp(i, FRQ, n)  { if((i)%((n)*(FRQ))==0) printf("%d\n", (i)/(FRQ)); } 
#define dispx(i, FRQ, n)  { disp(i,FRQ,n);  if(kbhit()&&getch()=='x') exit(0); }   // kbhit & getch, waste time!

void ClassSizeDisp(int i=0);

// class define
class CGLV;
class CVect3;		class CMat3;		class CQuat;		class CVect;		class CMat;
class CEarth;		class CIMU;			class CSINS;		class CAVPInterp;	class CAligni0;
class CKalman;		class CSINSTDKF;	class CSINSGNSS;	class CSINSGNSSDR;	class CAlignkf;
class CAlignsv;		class CAligntrkang;	class CCAM;			class CCALLH;		class CVGHook;
class CRAvar;		class CVAR;			class CVARn;		class CIIR;			class CIIRV3;
class CMaxMin;		class CMaxMinn;		class CRMemory;		class CSmooth;		class CFileRdWt;
class CDR;			class CVAutoPOS;	class CPOS618;		class CAutoDrive;	class CSINSGNSSOD;
class CSGOClbt;		class CWzhold;		class CUartPP;		class CInterp;		class CFileLog;
class CVCFileFind;	class ConUart;		class CContLarge;	class CFileCfg;

// global variables
extern const CVect3	O31, One31, I31Z, Ipos, posNWPU;
extern const CQuat	qI;
extern const CMat3	I33, O33, One33;
extern const CVect  On1, O1n, Onen1;
extern CVect3		Vrbs;
extern CGLV			glv;
extern CFileLog		psinslog;
extern CFileRdWt	*pfDebug;
extern int			psinslasterror;

// function define
double  r2dm(double r);
double	dm2r(double dm);
BOOL	logtrigger(int n, double f0=1.0);
inline  BOOL IsZero(double f, double eps=EPS);
int		sign(double val, double eps=EPS);
double  randn(double mu, double sigma=1.0);
double	range(double val, double minVal, double maxVal);
double  maxn(const double *pd, int n);
double  minn(const double *pd, int n);
double  norm1(const double *pd, int n);  // 1-norm
double  norm(const double *pd, int n);   // 2-norm
double  normInf(const double *pd, int n);// inf-norm
double  attract(double f, double th=1.0, double center=0.0);
double  polyval(const double *p, int order, double x);
double	atan2Ex(double y, double x);
double  diffYaw(double yaw, double yaw0);
double	MKQt(double sR, double tau);
char*	timestr(int type=0, char *p=NULL);
char*	time2fname(void);
double	unixt2gpst(double ut, int leap=0);
int*	deci(int i, int *pi=NULL);		// decode decimal to 0~9 array
BOOL	chkhdr(const char *str, const char *hdr);
BYTE*	flipud(BYTE *p, int rows, int clmBytes);
void	deal32(float *pf, ...);
void	deal(double *pf, ...);
#define crosI(v1,v2)		(v1.j*v2.k-v1.k*v2.j)
#define crosJ(v1,v2)		(v1.k*v2.i-v1.i*v2.k)
#define crosK(v1,v2)		(v1.i*v2.j-v1.j*v2.i)
#define VEQU(vv,v1)			{vv.i=v1.i, vv.j=v1.j, vv.k=v1.k;}
#define VCROS(vv,v1,v2)		{double i=crosI(v1,v2), j=crosJ(v1,v2); vv.k=crosK(v1,v2); vv.i=i, vv.j=j;}
#define VDOT(f,v1,v2)		{f=v1.i*v2.i+v1.j*v2.j+v1.k*v2.k;}
#define VDOTMUL(vv,v1,v2)	{vv.i=v1.i*v2.j, vv.j=v1.j*v2.j, vv.k=v1.k*v2.k;}
#define VMULf(vv,v1,f)		{vv.i=v1.i*(f), vv.j=v1.j*(f), vv.k=v1.k*(f);}
#define VADD(vv,v1,v2)		{vv.i=v1.i+v2.i, vv.j=v1.j+v2.j, vv.k=v1.k+v2.k;}
#define VADDE(vv,v2)		{vv.i+=v2.i, vv.j+=v2.j, vv.k+=v2.k;}
#define VSUB(vv,v1,v2)		{vv.i=v1.i-v2.i, vv.j=v1.j-v2.j, vv.k=v1.k-v2.k;}
#define VSUBE(vv,v2)		{vv.i-=v2.i, vv.j-=v2.j, vv.k-=v2.k;}
#define VADDf(vv,v1,v2,f2)	{vv.i=v1.i+v2.i*(f2), vv.j=v1.j+v2.j*(f2), vv.k=v1.k+v2.k*(f2);}
#define VADDff(vv,v1,f1,v2,f2)	{vv.i=v1.i*(f1)+v2.i*(f2), vv.j=v1.j*(f1)+v2.j*(f2), vv.k=v1.k*(f1)+v2.k*(f2);}
#define MMULf(m,m1,f)		{m.e00=m1.e00*(f),m.e01=m1.e01*(f),m.e02=m1.e02*(f),m.e10=m1.e10*(f),m.e11=m1.e11*(f),m.e12=m1.e12*(f),m.e20=m1.e20*(f),m.e21=m1.e21*(f),m.e22=m1.e22*(f);}
#define swapt(a, b, tpe)  { tpe tmp=a; a=b; b=tmp; };
#define pow2(x)			((x)*(x))
#define asinEx(x)		asin(range(x, -1.0, 1.0))
#define acosEx(x)		acos(range(x, -1.0, 1.0))
#define hit0   (NULL)
#define hit1(t, t10, t11)   ( t10<t && t<=t11 )
#define hit2(t, t10, t11, t20, t21)   ( hit1(t,t10,t11) || hit1(t,t20,t21) )
#define hit3(t, t10, t11, t20, t21, t30, t31)   ( hit2(t,t10,t11,t20,t21) || hit1(t,t30,t31) )
#define hit4(t, t10, t11, t20, t21, t30, t31, t40, t41)   ( hit3(t, t10, t11, t20, t21, t30, t31) || hit1(t,t40,t41) )

// Matrix Max Dimension define
#define MMD		PSINS_MATRIX_MAX_DIM
#define MMD2	(MMD*MMD)

class CGLV
{
public:
	double Re, f, g0, wie;										// the Earth's parameters
	double e, e2, ep, ep2, Rp;
	double mg, ug, deg, min, sec, hur, ppm, ppmpsh;				// commonly used units
	double dps, dph, dpsh, dphpsh, dph2, dphpg, ugpsh, ugpsHz, ugpg2, mpsh, mpspsh, secpsh;

	CGLV(double Re=RE, double f=FF, double g0=G0);
#ifdef PSINS_IO_FILE
	clock_t	t0;
	int toc(BOOL disp=0);
	~CGLV(void);
#endif
};

#ifdef PSINS_COMPLEX
class CComplex
{
public:
	double a, b;  // CComplex: z=a+bi
	CComplex(void) {};
	CComplex(double a0, double b0=0.0);
	CComplex operator+(const CComplex &z) const;		// complex addition
	CComplex operator+(double a0) const;
	CComplex operator-(const CComplex &z) const;		// complex subtraction
	CComplex operator-(double a0) const;
	CComplex operator*(const CComplex &z) const;		// complex multiplication
	CComplex operator*(double a0) const;
	CComplex operator/(const CComplex &z) const;		// complex divide
	CComplex operator/(double a0) const;
	CComplex& operator=(double a0);						// equal to a real
};
CComplex operator+(double a0, const CComplex &z);
CComplex operator-(double a0, const CComplex &z);
CComplex operator*(double a0, const CComplex &z);
CComplex operator/(double a0, const CComplex &z);
CComplex operator-(const CComplex &z);		// minus
CComplex operator~(const CComplex &z);		// complex conjugate
double real(const CComplex &z);				// real
double img(const CComplex &z);				// image
double norm(const CComplex &z);				// norm
double arg(const CComplex &z);				// argument
CComplex pow(const CComplex &z, double k);	// z^k
CComplex sqrt(const CComplex &z);
CVect3 m33abc(const CMat3 &m);
CVect3 realrt3(double a, double b, double c);	// real roots for x^3+ax^2+bx+c=0;
CVect3 ShengJin(double a, double b, double c);	// real roots for x^3+ax^2+bx+c=0;
#endif  // PSINS_COMPLEX

class CVect3 
{
public:
	double i, j, k;

	CVect3(void);
	CVect3(double xyz);
	CVect3(double xx, double yy, double zz);
	CVect3(const double *pdata);
	CVect3(const float *pdata);
	CVect3(const int *pdata, double f=1.0);

	CVect3& operator=(double f);							// every element equal to a same double
	CVect3& operator=(const double *pf);					// vector equal to a array
	CVect3 operator+(const CVect3 &v) const;				// vector addition
	CVect3 operator-(const CVect3 &v) const;				// vector subtraction
	CVect3 operator*(const CVect3 &v) const;				// vector cross multiplication
	CVect3 operator*(const CMat3 &m) const;					// row-vector multiply matrix
	CVect3 operator*(double f) const;						// vector multiply scale
	CVect3 operator/(double f) const;						// vector divide scale
	CVect3 operator/(const CVect3 &v) const;				// vector divide vect3 element by element
	CVect3& operator+=(const CVect3 &v);					// vector addition
	CVect3& operator-=(const CVect3 &v);					// vector subtraction
	CVect3& operator*=(double f);							// vector multiply scale
	CVect3& operator/=(double f);							// vector divide scale
	CVect3& operator/=(const CVect3 &v);					// vector divide vect3 element by element
	double& operator()(int r);								// vector element
};
BOOL IsZero(const CVect3 &v, double eps=EPS);		// psinsassert if all elements are zeros
BOOL IsZeroXY(const CVect3 &v, double eps=EPS);		// psinsassert if x&&y-elements are zeros
BOOL IsNaN(const CVect3 &v);						// psinsassert if any element is NaN
BOOL IsRad(const CVect3 &v, int n=2);				// if att or pos is rad, within [-pi,2*pi]
double AccScale(const CVect3 &mfb, double ts, double g=G0);
CVect3 operator*(double f, const CVect3 &v);		// scale multiply vector
double crossXY(const CVect3 &v1, const CVect3 &v2);
CVect3 operator-(const CVect3 &v);				// minus
CMat3 vxv(const CVect3 &v1, const CVect3 &v2);	// column-vector multiply row-vector, v1*v2'
CVect3 abs(const CVect3 &v);						// abs for each element
inline double absp(double val);
CVect3 absp(const CVect3 &v);
CVect3 maxabs(const CVect3 &v1, const CVect3 &v2);	// max_abs for each element
double norm(const CVect3 &v);					// vector norm
double normlize(CVect3 *v);						// vector normlize
double normInf(const CVect3 &v);					// vector inf-norm
double normXY(const CVect3 &v);					// vector norm of X & Y components
double normXYInf(const CVect3 &v);				// vector inf-norm of X & Y components
CVect3 sqrt(const CVect3 &v);					// sqrt for each element
CVect3 pow(const CVect3 &v, int k=2);				// power for each element
double dot(const CVect3 &v1, const CVect3 &v2);	// vector dot multiplication
CVect3 dotmul(const CVect3 &v1, const CVect3 &v2);	// vector dot multiplication '.*'
CVect3 dotdiv(const CVect3 &v1, const CVect3 &v2);	// vector dot divide './'
CMat3 a2mat(const CVect3 &att);					// Euler angles to DCM 
CVect3 m2att(const CMat3 &Cnb);					// DCM to Euler angles 
CVect3 m2attr(const CMat3 &Cnb);					// DCM to reversed Euler angles (in 3-2-1 rotation sequence)
CVect3 q2attr(const CQuat &qnb);					// Qnb to reversed Euler angles (in 3-2-1 rotation sequence)
CQuat a2qua(double pitch, double roll, double yaw);	// Euler angles to quaternion
CQuat a2qua(const CVect3 &att);					// Euler angles to quaternion
CMat3 ar2mat(const CVect3 &attr);				// reversed Euler angles to DCM
CQuat ar2qua(const CVect3 &attr);				// reversed Euler angles to Qnb
CVect3 q2att(const CQuat &qnb);					// quaternion to Euler angles 
CQuat rv2q(const CVect3 &rv);					// rotation vector to quaternion
CMat3 rv2m(const CVect3 &rv);					// rotation vector to DCM
CVect3 q2rv(const CQuat &q);						// quaternion to rotation vector
CMat3 askew(const CVect3 &v);					// askew matrix;
double sinAng(const CVect3 &v1, const CVect3 &v2, const CVect3 &v0=O31); // |sin(angle(v1,v2))|
CVect3 rotz(const CVect3 &v, double angle);
CMat3 pos2Cen(const CVect3 &pos);				// to geographical position matrix
CVect3 pp2vn(const CVect3 &pos1, const CVect3 &pos0, double ts=1.0, CEarth *pEth=NULL);  // position difference to velocity
CVect3 pp2att(const CVect3 &pos1, const CVect3 &pos0);  // position difference to attitude
CVect3 MKQt(const CVect3 &sR, const CVect3 &tau);// first order Markov white-noise variance calculation
CVect3 sv2att(const CVect3 &fb, double yaw0=0.0, const CVect3 &fn=I31Z);  // level attitude determination using single-vector
CVect3 dv2att(const CVect3 &va1, const CVect3 &va2, const CVect3 &vb1, const CVect3 &vb2);  // attitude determination using double-vector
CVect3 mv2att(int n, const CVect3 *vai, const CVect3 *vbi, ...);  // attitude determination using multiple-vector
CVect3 vn2att(const CVect3 &vn);  // trans ENU velocity to attitude (pitch & yaw)
CVect3 atss(CVect3 &att, CVect3 &vn);  // angles of attack & sideslip
CVect3 Alignsb(const CVect3 &wmm, const CVect3 &vmm, double lat);  // align in static-base
CVect3 Alignsb(const CVect3 &wmm, const CVect3 &vmm, const CVect3 &pos);
double MagYaw(const CVect3 &mag, const CVect3 &att, double declination=0.0);
CVect3 xyz2blh(const CVect3 &xyz);				// ECEF X/Y/Z to latitude/longitude/height
CVect3 blh2xyz(const CVect3 &blh);				// latitude/longitude/height to ECEF X/Y/Z 
CVect3 Vxyz2enu(const CVect3 &Vxyz, const CVect3 &pos);  // ECEF Vx/Vy/Vz to Ve/Vn/Vu
CVect3 randn(const CVect3 &mu, const CVect3 &sigma=One31);
CVect3 v2double(double f);
CVect3 v3double(double f);
void v5double(double f, CVect3 &v1, CVect3 &v2);
void v2flt(float *pf, const CVect3 *pv, ...);  // v2flt(pf, v1, v2, ..., NULL);
CVect3 sort(const CVect3 &v);
double median(const CVect3 &v);
double median(const double &f1, const double &f2, const double &f3);
CVect3 dm2r(const CVect3 &v, int typ=2);
CVect3 v3mmm(const CVect3 &p1, const CVect3 &p2, const CVect3 &p3, CVect3 *pmm=NULL, BOOL isMean=0, const CVect3 &th=One31);  //3-vect max-median-min
CVect3 fopp(const CVect3 &a, const CVect3 &b, const CVect3 &c); // foot of a perpendicular
CVect3 fopp(const CVect3 &a, const CVect3 &b, const CVect3 &c, const CVect3 &d); // foot of a perpendicular
CVect3 tp2att(const CVect3 &a, const CVect3 &b, const CVect3 &c); // triad point to attitude
CVect3 attract(const CVect3 &v, const CVect3 &th=One31, const CVect3 &center=O31);
CVect3 addw(const CVect3 &X1, const CVect3 &X2, double w1, double w2=INF);  // weighted addition, w1*X1+w2*X2
CVect3 ff2muxy(const CVect3 &f0, const CVect3 &f1, const char *dir0=NULL, const char *dir1=NULL);
CVect3 ff2mu(const CVect3 &f0, const CVect3 &f1, double uz=0.0);

class CQuat
{
public:
	double q0, q1, q2, q3;

	CQuat(void);
	CQuat(double qq0, double qq1=0.0, double qq2=0.0, double qq3=0.0);
	CQuat(double qq0, const CVect3 &qqv);
	CQuat(const double *pdata);

	CQuat operator+(const CVect3 &phi) const;	// true quaternion add misalign angles
	CQuat operator-(const CVect3 &phi) const;	// calculated quaternion delete misalign angles
	CVect3 operator-(const CQuat &quat) const;	// get misalign angles from calculated quaternion & true quaternion
	CQuat operator*(const CQuat &q) const;		// quaternion multiplication
	CVect3 operator*(const CVect3 &v) const;	// quaternion multiply vector
	CQuat& operator*=(const CQuat &q);			// quaternion multiplication
	CQuat& operator-=(const CVect3 &phi);		// calculated quaternion delete misalign angles
	void SetYaw(double yaw=0.0);				// set Euler angles to designated yaw
};
double normlize(CQuat *q);				// quaternion norm
CQuat operator~(const CQuat &q);		// quaternion conjugate
CVect3 qq2phi(const CQuat &qcalcu, const CQuat &qreal); // phi = qcalcu - qreal
CQuat addmu(const CQuat &q, const CVect3 &mu); // qreal = qcalcu + mu
CQuat UpDown(const CQuat &q);		// Up-Down the quaternion represented attitide

class CMat3 
{
public:
	double e00, e01, e02, e10, e11, e12, e20, e21, e22;

	CMat3(void);
	CMat3(double xyz);
	CMat3(const double *pxyz);
	CMat3(const float *pxyz);
	CMat3(double xx, double yy, double zz);
	CMat3(double xx, double xy, double xz,
		  double yx, double yy, double yz,
		  double zx, double zy, double zz );
	CMat3(const CVect3 &v0, const CVect3 &v1, const CVect3 &v2, BOOL isrow=1);  // M = [v0; v1; v2]

	CMat3 operator+(const CMat3 &m) const;					// matrix addition
	CMat3 operator-(const CMat3 &m) const;					// matrix subtraction
	CMat3 operator*(const CMat3 &m) const;					// matrix multiplication
	CMat3 operator*(double f) const;						// matrix multiply scale
	CVect3 operator*(const CVect3 &v) const;				// matrix multiply vector
	CMat3& operator+=(const CMat3 &m);						// matrix +=
	CMat3 operator+(const CVect3 &v) const;					// matrix addition
	CMat3& operator+=(const CVect3 &v);						// matrix + diag(vector)
	double& operator()(int r, int c);						// vector element
	void SetRow(int i, const CVect3 &v);					// set i-row from vector
	void SetClm(int i, const CVect3 &v);					// set i-column from vector
	CVect3 GetRow(int i) const;								// get i-row from matrix
	CVect3 GetClm(int i) const;								// get i-column from matrix
};
CMat3 Rot(double angle, char axis);				// rotation by x/y/z axis with angle
CMat3 rcijk(const CMat3 &m, int ijk);			// re-arrange row/clm indexed by ijk
CMat3 operator-(const CMat3 &m);					// minus
CMat3 operator~(const CMat3 &m);					// matrix transposition
CMat3 operator*(double f, const CMat3 &m);		// scale multiply matrix
void symmetry(CMat3 &m);							// matrix symmetrization
CMat3 pow(const CMat3 &m, int k);				// k^th power
double trace(const CMat3 &m);					// matrix trace
double det(const CMat3 &m);						// matrix determinat
CMat3 adj(const CMat3 &m);						// 3x3 adjoint matrix
CMat3 inv(const CMat3 &m);						// 3x3 matrix inverse
CVect3 diag(const CMat3 &m);						// the diagonal of a matrix
CMat3 diag(const CVect3 &v);						// diagonal matrix
CMat3 diag(double ii, double jj=INF, double kk=INF);	// diagonal matrix
CMat3 askew(const CMat3 &m, int I=0);				// askew matrix;
CMat3 dotmul(const CMat3 &m1, const CMat3 &m2);	// m = m1.*m2;
CMat3 MMT(const CMat3 &m1, const CMat3 &m2=I33);		// m=m1*m2^T
double trMMT(const CMat3 &m1, const CMat3 &m2=I33);	// trace(m1*m2^T)
double norm(const CMat3 &m);						// matrix norm
CVect3 m2rv(const CMat3 &Cnb);					// DCM to rotation vector
CQuat m2qua(const CMat3 &Cnb);					// DCM to quaternion
CMat3 q2mat(const CQuat &qnb);					// attitude quaternion to DCM
CMat3 Ka2Cba(const CMat3 &Ka, CVect3 &Sfa=Vrbs);
CMat3	Cba2Ka(const CMat3 &Cba, const CVect3 &Sfa=One31);
//	 CMat3 Ka2Cba(const CMat3 &Ka, CVect3 *pSfa=NULL);
//	 CMat3 Cba2Ka(const CMat3 &Cba, const CVect3 Sfa=One31);
CMat3 sfoam(const CMat3 &B, int iter=50);			// Supper Fast Optimal Attitude Matrix(SFOAM)
CMat3 randn(const CMat3 &mu, const double &sigma=1.0);// random 3x3 matrix
void Ka22Kpn(const CVect3 &Ka1, const CVect3 &Ka2, CVect3 &Kap, CVect3 &Kan);
void Kpn2Ka2(const CVect3 &Kap, const CVect3 &Kan, CVect3 &Ka1, CVect3 &Ka2);
void KgMdf(CMat3 &Kg, const double *dKg, int typ=0);
void KaMdf(CMat3 &Ka, const double *dKa, int typ=0);

class CVect
{
public:
	int row, clm, rc;
	double dd[MMD];

	CVect(void);
	CVect(int row0, int clm0=1);
	CVect(int row0, double f);
	CVect(int row0, double f, double f1, ...);
	CVect(int row0, const double *pf);
	CVect(const CVect3 &v);
	CVect(const CVect3 &v1, const CVect3 v2);

	void Clear(void);
	void Reset(int row0, int clm0=1);
	void Set(double f, ...);
	void Set2(double f, ...);
	void SetAscend(double f0, double df);
	void SetVect3(int i, const CVect3 &v);
	void Set2Vect3(int i, const CVect3 &v);		// pow2
	void SetBit(unsigned int bit, double f);	// set element to f by bit mask
	void SetBit(unsigned int bit, const CVect3 &v);
	void Seti2j(int i, int j=MMD-1, double val=0.0);  // dd[i]~dd[j] = val
	CVect3 GetVect3(int i) const;
	CVect operator+(const CVect &v) const;		// vector addition
	CVect operator-(const CVect &v) const;		// vector subtraction
	CVect operator*(double f) const;			// vector multiply scale
	CVect& operator=(double f);					// every element equal to a same double
	CVect& operator=(const double *pf);			// vector equal to a array
	CVect& operator=(const CMat3 &m);			// vector equal to matrix 3x3
	CVect& operator+=(const CVect &v);			// vector addition
	CVect& operator-=(const CVect &v);			// vector subtraction
	CVect& operator*=(double f);				// vector multiply scale
	CVect operator*(const CMat &m) const;		// row-vector multiply matrix
	CMat operator*(const CVect &v) const;		// 1xn vector multiply nx1 vector, or nx1 vector multiply 1xn vector
	double& operator()(int r);					// vector element
};
double dot(const CVect &v1, const CVect &v2);	// vector dot multiplication
CVect dotmul(const CVect &v1, const CVect &v2);	// vector dot multiplication '.*'
CVect operator-(const CVect &v);		// minus
CVect operator~(const CVect &v);		// vector transposition
CVect abs(const CVect &v);			// vector abs for each element
void neg(CVect &v, const int idx[], int n=-1);		// negative some elements
void enlarge(CVect &v, double f, const int idx[], int n=-1);	// enlarge some elements
double norm(const CVect &v);			// vector norm
double normInf(const CVect &v);		// inf-norm
CVect pow(const CVect &v, int k=2);	// vector element power
CVect randn(const CVect &mu, const CVect &sigma=Onen1); // random nx1 vector
double mean(const CVect &v);
CVect sort(const CVect &v);

class CMat
{
public:
	int row, clm, rc;
	double dd[MMD2];

	CMat(void);
	CMat(int row0, int clm0=0);
	CMat(int row0, int clm0, double f);
	CMat(int row0, int clm0, double f, double f1, ...);
	CMat(int row0, int clm0, const double *pf);
	CMat(int clm0, const CVect *pv, ...);

	void Clear(double f=0.0);
	void ClearRow(int i, double f=0.0);
	void ClearClm(int j, double f=0.0);
	void ClearRC(int i, int j=-1, double f=0.0);
	void Reset(int row0, int clm0);
	void SetDiag(double f, ...);
	void SetDiag2(double f, ...);
	void SetAscend(double f0=0.0, double df=1.0);
	CMat operator+(const CMat &m) const;				// matrix addition
	CMat operator-(const CMat &m) const;				// matrix subtraction
	CMat operator*(double f) const;						// matrix multiply scale
	CVect operator*(const CVect &v) const;				// matrix multiply vector
	CMat operator*(const CMat &m) const;				// matrix multiplication
	CMat operator*(const CMat3 &m) const;				// matrix multiplication
	CMat& operator=(double f);							// every element equal to a same double
	CMat& operator+=(const CMat &m0);					// matrix addition
	CMat& operator+=(const CVect &v);					// matrix + diag(vector)
	CMat& operator-=(const CMat &m0);					// matrix subtraction
	CMat& operator*=(double f);							// matrix multiply scale
	CMat& operator++();									// 1.0 + diagonal
	double& operator()(int r, int c=-1);				// get element m(r,c)
	void ZeroRow(int i);								// set i-row to 0
	void ZeroClm(int j);								// set j-column to 0
	void ZeroRC(int i, int j=-1);						// set i-row & j-column to 0
	void SetRow(int i, double f, ...);					// set i-row from n-double
	void SetRow(int i, const CVect &v);					// set i-row from vector
	void SetClm(int j, double f, ...);					// set j-column from n-double
	void SetClm(int j, const CVect &v);					// set j-column from vector
	CVect GetRow(int i) const;							// get i-row from matrix
	void GetRow(CVect &v, int i);
	CVect GetClm(int j) const;							// get j-column from matrix
	CMat GetClm(int c1, int c2, ...) const;				// get c1,c2...-column from matrix
	void GetClm(CVect &v, int j);
	void SetRowVect3(int i, int j, const CVect3 &v);	// set i-row&j...(j+2)-column from CVect3
	void SetRowVect3(int i, int j, const CVect3 &v, const CVect3 &v1);
	void SetRowVect3(int i, int j, const CVect3 &v, const CVect3 &v1, const CVect3 &v2);
	void SetClmVect3(int i, int j, const CVect3 &v);	// set i...(i+2)-row&j-column from CVect3
	void SetClmVect3(int i, int j, const CVect3 &v, const CVect3 &v1);
	void SetClmVect3(int i, int j, const CVect3 &v, const CVect3 &v1, const CVect3 &v2);
	CVect3 GetRowVect3(int i, int j) const;				// get i-row&j...(j+2)-column from matrix
	CVect3 GetClmVect3(int i, int j) const;				// get i...(i+2)-row&j-column from matrix
	void SetDiagVect3(int i, int j, const CVect3 &v);	// m(i,j)=v.i, m(i+1,j+1)=v.j, m(i+2,j+2)=v.k;
	CVect3 GetDiagVect3(int i, int j=-1) const;			// return CVect3(m(i,j), m(i+1,j+1), m(i+2,j+2))
	void SetAskew(int i, int j, const CVect3 &v);		// set i...(i+2)-row&j...(j+2)-comumn from askew CVect3
	void SetMat3(int i, int j, const CMat3 &m);			// set i...(i+2)-row&j...(j+2)-comumn from CMat3
	void SetMat3(int i, int j, const CMat3 &m, const CMat3 &m1);
	void SetMat3(int i, int j, const CMat3 &m, const CMat3 &m1, const CMat3 &m2);
	CMat3 GetMat3(int i, int j=-1) const;				// get CMat3 from i...(i+2)-row&j...(j+2)-comumn
	void SubAddMat3(int i, int j, const CMat3 &m);		// add i...(i+2)-row&j...(j+2)-comumn with CMat3 m
#ifdef PSINS_MAT_COUNT
	static int iCount, iMax;
	~CMat(void);
#endif
};
CMat operator~(const CMat &m);				// matrix transposition
void symmetry(CMat &m);						// matrix symmetrization
double trace(const CMat &m);					// matrix trace
double norm1(const CMat &m);					// 1-norm
double normInf(const CMat &m);				// inf-norm
CMat dotmul(const CMat &m1, const CMat &m2);	// matrix dot multiplication '.*'
CVect diag(const CMat &m);					// diagonal of a matrix
CMat diag(const CVect &v);					// diagonal matrix
CMat eye(int n);
CMat MatExam(int i, int j=0);
CMat inv4(const CMat &m);					// 4x4 matrix inverse
CMat inv6(const CMat &m);					// 6x6 matrix inverse
CVect lss(const CMat &A, const CVect &y);		// least square solusion
void RowMul(CMat &m, const CMat &m0, const CMat &m1, int r, int fast=0); // m(r,:)=m0(r,:)*m1
void RowMulT(CMat &m, const CMat &m0, const CMat &m1, int r, int fast=0); // m(r,:)=m0(r,:)*m1'
void DVMDVafa(const CVect &V, CMat &M, double afa=1.0);	// M = diag(V)*M*diag(V)*afa
CMat randn(const CMat &mu, const double &sigma=1.0);

class CPolyfit
{
public:
	int n, inHk, byUD;
	double ts, tk, Rk, Hk[5], Kk[5], Xk[5], Pk[5][5]; // a0+a1*x+a2*x^2+a3*x^3+a4*x^4
	double *U, D[5]; // for UDUT meas update

	CPolyfit(void);
	virtual void Init(double ts0, int n0=4, double spii=1.0e3);
	void SetP(double spii, ...);
	void SetUD(double spii, ...);
	void SetHk(double hi, ...);
	void UpdateP(double afa);
	void Update(double Zk, double afa=1.0);
	double eval(double t);
};

class CPolyfit3:public CPolyfit
{
public:
	CVect3 Xkv[5];

	CPolyfit3(void);
	virtual void Init(double ts0, int n0=4, double spii=1.0e3);
	void Update(const CVect3 &Zk, double afa=1.0);
	CVect3 eval(double t);
};

class CRAvar
{
public:
	#define RAMAX MMD
	int nR0, Rmaxcount[RAMAX], Rmaxflag[RAMAX];
	double ts, R0[RAMAX], Rmax[RAMAX], Rmin[RAMAX], tau[RAMAX], r0[RAMAX];

	CRAvar(void);
	CRAvar(int nR0, int maxCount0=2);
	void set(double r0, double tau, double rmax=0.0, double rmin=0.0, int i=0);
	void set(const CVect3 &r0, const CVect3 &tau, const CVect3 &rmax=O31, const CVect3 &rmin=O31);
	void set(const CVect &r0, const CVect &tau, const CVect &rmax=On1, const CVect &rmin=On1);
	void Update(double r, double ts, int i=0);
	void Update(const CVect3 &r, double ts);
	void Update(const CVect &r, double ts);
	double operator()(int k) const;			// get element sqrt(R0(k))
};

class CVAR
{
public:
	#define VARMAX 50
	int ipush, imax;
	double array[VARMAX], mean, var;

	CVAR(int imax0=10, double data0=0.0);
	double Update(double data, BOOL isvar=TRUE);
};

class CVARn {
public:
	int row, clm, idxpush, rowcnt;
	double **pData, *pd, *Sx, *Sx2, *mx, *stdx;  // sum(x), sum(x^2), mean(x), std(x)
	double stdsf;  // std scalefactor
	CVARn(void);
	CVARn(int row0, int clm0);
	~CVARn(void);
	void Reset(void);
	BOOL Update(const double *pd);
	BOOL Update(double f, ...);
};

class CContLarge {  // data continous to be large value
public:
	double large, dt, t0;
	int cnt, cnti;
	CContLarge(void);
	CContLarge(double large0, double dt0=1.5, int cnt0=5);
	BOOL Update(double val, double t1=INF);
};

class CAbnomalCnt {  // data abnomal counter
public:
	int cntMax, cnt, abnFlag;
	double t0, tlast, valMax, valMin;
	CAbnomalCnt(void);
	CAbnomalCnt(int cntMax0, double tlast0, double valMax0, double valMin0=-INF);
	BOOL Update(double val, double t=0.0);
};

class CWzhold {
public:
	double maxw, ts, t, T, tstop, meanw, meanwpre, meanwprepre, val;
	int cntNP, cntNi, cntPi, big, retres;
	CWzhold(void);
	void Init(double maxw0, double ts0, double T0=10.0, int cntNP0=10);
	void Reset(void);
	int Update(double wz);
};

class CMaxMin {
public:
	float max0, min0, maxpre0, minpre0,
		maxCur, minCur, maxpreCur, minpreCur,
		maxRes, minRes, maxpreRes, minpreRes, diffRes, diffpreRes, meanRes, sumRes, maxabsRes;
	int cnt0, cntCur, cntpreCur, flag;
	CMaxMin(int cnt00=100, int pre00=0, float f0=0.0f);
	void Init(int cnt00=100, int pre00=0, float f0=0.0f);
	void Restart(void);
	int Update(float f);
};

class CMaxMinn {
public:
	int n, flag;
	CMaxMin mm[MMD];
	CMaxMinn(int n0=3, int cnt00=100, int pre00=0, float f0=0.0f);
	void Init(int n0=3, int cnt00=100, int pre00=0, float f0=0.0f);
	void Restart(void);
	int Update(float f, ...);
	int Update(const CVect3 &v);
	int Update(const CVect3 &v1, const CVect3 &v2);
	int Update(const CVect3 &v1, const CVect3 &v2, const CVect3 &v3);
	int Update(const CVect3 &v1, const CVect3 &v2, const CVect3 &v3, const CVect3 &v4);
	int Update(const CVect3 &v1, const CVect3 &v2, const CVect3 &v3, const CVect3 &v4, const CVect3 &v5);
	float ResFloat(int i, int minmeanmaxFlag);
	CVect3 ResVect3(int i=0, int minmeanmaxFlag=0);
};

class CEarth
{
public:
	double a, b;
	double f, e, e2;
	double wie;

	double sl, sl2, sl4, cl, tl, RMh, RNh, clRNh, f_RMh, f_RNh, f_clRNh;
	CVect3 pos0, pos, vn, wnie, wnen, wnin, wnnin, gn, gcc, *pgn;

	CEarth(double a0=RE, double f0=FF, double g0=G0);
	void Init(double a0=RE, double f0=FF, double g0=G0);
	void Update(const CVect3 &pos, const CVect3 &vn=O31, int isMemsgrade=0);
	CVect3 vn2dpos(const CVect3 &vn, double ts=1.0) const;
	void vn2dpos(CVect3 &dpos, const CVect3 &vn, double ts) const;
};
double pos2g(const CVect3 &pos);
CVect3 pos2wnie(const CVect3 &pos);

class CEGM  // Earth Gravitational Model 
{
public:
	CEGM(void);
	int Init(const char *fileModel);
	int Init(double GM0, double Re0, double ff0, double wie0, double *pC0, double *pS0, int N0);
	~CEGM(void);
	void Update(double *blh, double *gn, int degree=-1);
};

class CIMU
{
public:
	CMat3 Kg; CVect3 eb; CMat3 Ka; CVect3 db, Kapn, Ka2, lvx, lvy, lvz, Taux, Tauy, Tauz; double tGA;  // Do not modify this line!!!
	CMat3 *pKga, *pgSens, gSens, *pgSens2, gSens2, *pgSensX, gSensX;  // gyro g-sensitivity
	CVect3 Sfg, Sfa, *pSf, *pKapn, *pKa2, *pTau;  // acc quadratic nonlinearity
	CVect3 *plv, wb_1;  // inner lever-arm
	CMat3 Cba, iTCba, SSx, SSy, SSz, *pCba;
	CVect3 Q11, Q12, Q13, Q21, Q22, Q23, Q31, Q32, Q33;
	char *prfu, rfu[3];
	int nSamples, iTemp;
	double tk, nts, _nts, smmT, Temp, *pTempArray;
	bool preFirst, onePlusPre, preWb, compensated;
	CVect3 phim, dvbm, wmm, vmm, swmm, svmm, wm_1, vm_1;

	CIMU(void);
	void Reset(void);
	void SetSf(const CVect3 &Sfg0=One31, const CVect3 &Sfa0=One31);
	void SetTemp(double *tempArray0, int type=1);
	void SetKga(const CMat3 &Kg0=I33, const CVect3 eb0=O31, const CMat3 &Ka0=I33, const CVect3 &db0=O31);
	void SetgSens(const CMat3 &gSens0, const CMat3 &gSens20=O33, const CMat3 &gSensX0=O33);
	void SetKapn(const CVect3 &Kapn0=O31);
	void SetKa2(const CVect3 &Ka20=O31);
	void SetLvtGA(const CVect3 &lvx0=O31, const CVect3 &lvy0=O31, const CVect3 &lvz0=O31, double tGA0=0.0);
	void SetTau(const CVect3 &taux0=O31, const CVect3 &tauy0=O31, const CVect3 &tauz0=O31);
	void SetCba(const CMat3 &Cba0=I33);
	void SetRFU(const char *rfu0);
	double GetMeanwf(CVect3 &wib, CVect3 &fsf, BOOL reset=1);
	double Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts);
};
void IMURFU(CVect3 *pwm, int nSamples, const char *str="X");
void IMURFU(CVect3 *pwm, CVect3 *pvm, int nSamples, const char *str="X");
void IMUStatic(CVect3 &wm, CVect3 &vm, CVect3 &att0, CVect3 &pos0, double ts=1.0);
CMat lsclbt(CMat &wfb, CMat &wfn);

class CIMUInc
{
public:
	int diGx, diGy, diGz, diAx, diAy, diAz;
	int iTotalGx0, iTotlaGy0, iTotalGz0, iTotalAx0, iTotalAy0, iTotalAz0;
	int iTotalGx, iTotalGy, iTotalGz, iTotalAx, iTotalAy, iTotalAz;
	double fTotalGx, fTotalGy, fTotalGz, fTotalAx, fTotalAy, fTotalAz, gScale, aScale;

	CIMUInc(double gScale=0.1*SEC, double aScale=0.001);
	void Init(double gScale=0.1*SEC, double aScale=0.001);
	void Update(const CVect3 &wm, const CVect3 &vm);
};

class CIMUInv
{
public:
	CVect3 wm, vm, wm0, vm0, att, vn, pos, vn0, pos0;
	CMat3 Cbn0;
	CEarth eth;
	double ts, tk;
	BOOL isFirst;

	CIMUInv(const CVect3 &att00, const CVect3 &vn00, const CVect3 &pos00, double ts0, double tk0=0.0);
	void Update(const CVect3 &att, const CVect3 &pos);
};

class CSINS	// sizeof(CSINS)~=3k bytes
{
public:
	double ts, nts, tk, tpps, mvnt, mvnT, lvlT, dist, velMax, hgtMin, hgtMax, latMax, afabar;
	int mvnk, iReverse;
	CEarth eth;
	CIMU imu;
	CQuat qnb;
	CMat3 Cnb, Cnb0, Cbn, mvnCnb0, Kg, Ka;
	CVect3 wib, fb, fn, an, anbar, web, webbar, wnb, att, vn, mvn, mvni, mvnmax, mvnmin, lvlVn0, vb, pos, eb, db, Ka2, tauGyro, tauAcc, _betaGyro, _betaAcc;
	CMat3 Maa, Mav, Map, Mva, Mvv, Mvp, Mpv, Mpp;	// for etm
	CVect3 lvr, vnL, posL; CMat3 CW, MpvCnb;		// for lever arm
	CQuat qnbE; CVect3 attE, vnE, posE;				// for extrapolation
	CMaxMin mmwb, mmfb;
	BOOL isOpenloop, isMemsgrade, isNocompasseffect, isOutlever;

	CSINS(double yaw0, const CVect3 &pos0=O31, double tk0=0.0);
	CSINS(const CVect3 &att0, const CVect3 &vn0=O31, const CVect3 &pos0=O31, double tk0=0.0);
	CSINS(const CQuat &qnb0=qI, const CVect3 &vn0=O31, const CVect3 &pos0=O31, double tk0=0.0);
	void Init(const CVect3 &att0, const CVect3 &vn0=O31, const CVect3 &pos0=O31, double tk0=0.0);    // initialization using quat attitude, velocity & position
	void Init(const CQuat &qnb0=qI, const CVect3 &vn0=O31, const CVect3 &pos0=O31, double tk0=0.0);    // initialization using quat attitude, velocity & position
	void SetTauGA(const CVect3 &tauG, const CVect3 &tauA);
	void Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts);		// SINS update using Gyro&Acc samples
	void Extrap(const CVect3 &wm=O31, const CVect3 &vm=O31, double ts=0.0);			// SINS fast extrapolation using 1 Gyro&Acc sample
	void Extrap(double extts);			// SINS fast extrapolation using previous Gyro&Acc sample
	void lever(const CVect3 &dL=O31, CVect3 *ppos=NULL, CVect3* pvn=NULL);		// lever arm
	void lever2(const CVect3 &dL, CVect3 *ppos, CVect3 *pvn=NULL, const CVect3 *ppos0=NULL, const CVect3 *pvn0=NULL);
	void atss(double *attack, double *sideslip);  // angles of attack & sideslip
	void etm(void);							// SINS error transform matrix coefficients
	void AddErr(const CVect3 &phi, const CVect3 &dvn=O31, const CVect3 &dpos=O31);
	void AddErr(double phiU, const CVect3 &dvn=O31, const CVect3 &dpos=O31);
	void Leveling(int flag);
	void Reverse(void);
	BOOL isPPS(double pps=1.0);
	void DebugStop(double t1, int absT=0, int ext=0);
};

class CDR	// dead reckoning
{
public:
	double tk, dist, Kod, velPre, velMax, velMin, afa, gck1, gck2, gck3;
	CEarth eth;
	CQuat qnb;
	CMat3 Cnb, Cbo;
	CVect3 att, vn, pos, wnc, vni, dpos;	
	CDR(void);
	void Init(const CSINS &sins, const CVect3 &kappa=CVect3(0,1,0));
	void Init(const CVect3 &att0, const CVect3 &pos0, const CVect3 &kappa=CVect3(0,1,0), double tk0=0.0);
	void Update(const CVect3 &wm, double dS, double ts, const CVect3 &vm=O31);
	void SetGCK(double Td);  // gyro-compass control coefficients K1,K2,K3
	void Leveling(const CVect3 &vm, double ts);
	CVect3 Calibrate(const CVect3 &pos0, const CVect3 &pos1, const CVect3 &pos1DR, double dist0=0.0);
};

class CCNS
{
public:
	double dUT1, dTAI, dTT, TT, era, gmst, gast, eps, dpsi, deps;
	CMat3 CP, CN, CW, Cie, Cns;
	CCNS(void);
	void SetdT(double dUT1=-0.5, double dTAI=37);
	void Setxyp(double xp=0.0, double yp=0.0);				// polar motion
	double JD(int year=2000, int month=1, int day=1, double hour=0.0);	// JulianDate calculation
	void Equinox(double TT);
	CMat3 Precmat(double TT);								// Luni-solar precession
	CMat3 Nutmat(double TT);								// Luni-solar nutation
	double GAST(double jd, double s);						// Greenwich Apparent Sidereal Time
	CMat3 GetCie(double jd, double s);
	CMat3 GetCns(const CQuat &qis, const CVect3 &pos, double t, const CMat3 &Cbs=I33);
};

class CAVPInterp
{
#define AVPINUM 20
public:
	double ts;
	int ipush, avpinum;
	CVect3 atti[AVPINUM], vni[AVPINUM], posi[AVPINUM];
	CVect3 att, vn, pos;
	CAVPInterp(void);
	void Init(const CSINS &sins, double ts, BOOL islever=0, int num=0);
	void Init(const CVect3 &att0, const CVect3 &vn0, const CVect3 &pos0, double ts, int num=0);
	void Push(const CSINS &sins, BOOL islever=0);
	void Push(const CVect3 &attk, const CVect3 &vnk=O31, const CVect3 &posk=O31);
	int Interp(double tpast, int avp=0x7);	// AVP interpolation, where -AVPINUM*ts<=tpast<=0
};

class CIIR
{
public:
	#define IIRnMax 6
	int n;
	double b[IIRnMax], a[IIRnMax], x[IIRnMax], y[IIRnMax];

	CIIR(void);
	CIIR(double *b0, double *a0, int n0);
	double Update(double x0);
};

class CIIRV3
{
public:
	CIIR iir0, iir1, iir2;
	CVect3 y;

	CIIRV3(void);
	CIIRV3(double *b0, double *a0, int n0,
		   double *b1=(double*)NULL, double *a1=(double*)NULL, int n1=0, 
		   double *b2=(double*)NULL, double *a2=(double*)NULL, int n2=0);
	CVect3 Update(const CVect3 &x);
};

class CAlignsb
{
public:
	double tk0, tk, yaw0;
	CVect3 att, pos0, wmm, vmm, eb, db;
	CEarth eth;
	CQuat qnb;

	CAlignsb(double lat0=0.0, double tk0=0.0);
	CAlignsb(const CVect3 &pos0, double tk0=0.0);
	void Init(double lat0, double tk0=0.0);
	void Init(const CVect3 &pos0, double tk0=0.0);
	void SetYaw(double yaw0);
	CQuat Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts);
};

class CAligni0
{
public:
	int velAid, t0, t1, t2;
	CVect3 pos0, vel0, wmm, vmm, vib0, vi0, Pib01, Pib02, Pi01, Pi02, tmpPib0, tmpPi0;
	CQuat qib0b;
	CEarth eth;
	CIMU imu;
	double tk;
	CQuat qnb0, qnb, qnbsb;

	CAligni0(const CVect3 &pos0=O31, const CVect3 &vel0=O31, int velAid=0);
	void Init(const CVect3 &pos0=O31, const CVect3 &vel0=O31, int velAid=0);
	CQuat Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts, const CVect3 &vel=O31);
};

class CAligni0fit
{
public:
	CVect3 pos0, vib0, pib0, vn0, vnt, xyzt;
	CQuat qib0b, qn0i0, qnb0, qnb, qnbsb;
	CEarth eth;
	CIMU imu;
	double tk;
	CPolyfit3 pfit4;

	CAligni0fit(void);
	CAligni0fit(const CVect3 &pos0);
	void Init(const CVect3 &pos0);
	CQuat Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts);
	CVect3 pi0t(double t);
	CVect3 pib0t(double t);
};

class CKalman
{
public:
	double kftk, zfdafa;
	int nq, nr;
	unsigned int kfcount, measflag, measflaglog, measmask;
	CMat Ft, Pk, Hk, Fading;
	CVect Xk, Zk, Zk_1, Qt, Rt, Rt0, Rset, rts, RtTau, measstop, measlost, Xmax, Pmax, Pmin, Pset, Zfd, Zfd0, Zmm0, Zmax, innoDiffMax, innoPre,
		Rmax, Rmin, Rbeta, Rb, Rstop,			// measurement noise R adaptive
		innoMax,				// innovation outlier
		FBTau, FBMax, FBOne, FBOne1, FBXk, FBTotal;	// feedback control
//	CVect Xk, Zk, Qt, Rt, rts, Xmax, Pmax, Pmin, Pset, Zmax, Zdiffmax, Zdiffpre,
//		innoOutlier,				// innovation outlier
//		FBTau, FBMax, FBOne, FBOne1, FBXk, FBTotal;	// feedback control
//	CVectf Zfd, Zfd0, Zmm0, RtTau, Rmax, measstop, measlost, Rmin, Rb, Rstop, Rbeta;
	int Rmaxcount[MMD], Rmaxcount0[MMD], innoMaxcount[MMD], innoMaxcount0, Zmmpk[MMD];
	CMaxMinn Zmm;

	CKalman(void);
	CKalman(int nq0, int nr0);
	void Init(int nq0, int nr0);				// initialize Qk,Rk,P0...
	void SetRmmbt(double rmin=0.1f, double rmax=INF, double b=INF, double tau=INF);
	void SetRmaxcount(int cnt=5);
	void SetInnoMaxcount(int cnt=5);
	void SetZmm(int zi, int pi, double zmm0, int cnt=10);
	void SetZmmVn(const CVect3 &zmm0, int cnt=10);
	void SetZmmPos(const CVect3 &zmm0, int cnt=10);
	virtual void SetFt(int nnq) = 0;			// process matrix setting
	virtual void SetHk(int nnq) = 0;			// measurement matrix setting
	virtual void SetMeas(void) = 0;				// set measurement
	virtual void Feedback(int nnq, double fbts);	// state feedback
	void FeedbackAll(void);
	void RtFading(int i, double fdts);			// Rt growing if no measurment
	void TimeUpdate(double kfts0, int fback=1);	// time update
	int MeasUpdate(double fading=1.0);			// measurement update
	int RAdaptive(int i, double r, double Pr);	// Rt adaptive
	void RPkFading(int i);						// multiple fading
	void ZmmPkSet(int i);						// Z_max_min for Pk setting
	void SetStatMask(unsigned int mask2, int k2=19, unsigned int mask1=017, int k1=15, unsigned int mask0=077777);	// state mask setting
	void SetMeasMask(unsigned int mask, int type=1);	// measurement mask setting
	void SetMeasFlag(unsigned int flag, int type=1);	// measurement flag setting
	void SetMeasStop(unsigned int meas, double stop=10.0f);
	void SetRadptStop(unsigned int meas, double stop=10.0f);
	void XPConstrain(void);						// Xk & Pk constrain: -Xmax<Xk<Xmax, Pmin<diag(Pk)<Pmax
	void PmaxPminCheck(void);
};
void MeasUD(double U[], double D[], const double H[], double R, double K[], int n);
void fusion(double *x1, double *p1, const double *x2, const double *p2,	int n=9, double *xf=NULL, double *pf=NULL);
void fusion(CVect3 &x1, CVect3 &p1, const CVect3 x2, const CVect3 p2);
void fusion(CVect3 &x1, CVect3 &p1, const CVect3 x2, const CVect3 p2, CVect3 &xf, CVect3 &pf);

class CSINSTDKF:public CKalman
{
public:
	double meantdts, tdts, Pz0, innovation, Kkp2y, Kkv2y;
	int iter, ifn, cststt, hi1[MMD], adptOKi, measRes, tdStep, maxStep, curOutStep, maxOutStep;  // cststt:first ConSTant STaTe index
	CMat Fk, Pk1; 
	CVect Pxz, Qk, Kk, Hi, tmeas;
	CVect3 meanfn;
	CSINS sins;
	unsigned int timcnt0, timcnt1, timcntmax;  // for calculation-burden analysis
	double burden;

	CSINSTDKF(void);
	CSINSTDKF(int nq0, int nr0);
	void Init(const CSINS &sins0);
	void TDReset(void);
	double Innovationi(int row);
	int TDUpdate(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts, int nStep=1);  // Time-Distributed Update
	void MeasUpdate(const CVect &Hi, double Ri, double Zi);
	void MarkovGyro(const CVect3 &tauG, const CVect3 &sRG, int stateeb=9);
	void MarkovAcc(const CVect3 &tauA, const CVect3 &sRA, int statedb=12);
	void SetYaw(double yaw, int statephi=0, int statedvn=3);
	void PSetVertCh(double sph, double spv=0.0, double spd=0.0);		// vertical channel P set
	double SetCalcuBurden(unsigned int timcnt, int itype);
	virtual void RTOutput(void) {};  // real-time output before KF update i.e. just after SINS update
	virtual void Miscellanous(void) {};
	virtual void SecretAttitude(void) {};
};

class CSINSGNSS:public CSINSTDKF	// sizeof(CSINSGNSS)~=21k bytes for 15-state KF
{
public:
	double kfts;
	double posGNSSdelay, vnGNSSdelay, yawGNSSdelay, dtGNSSdelay, dyawGNSS, *gnssLost;
	int yawHkRow, navStatus;
	CVect3 lvGNSS;
	CAVPInterp avpi;
	CVect3 *pphi, *pdvn, *pdpos, *peb, *pdb, *plvr, *pdkgz, *pdkaii, *pdkg1, *pdkg2, *pdkg3, *pdka1, *pdka23;
	double *pdT, *pdkgzz, *pddbz;

	CSINSGNSS(void);
	CSINSGNSS(int nq0, int nr0, double ts, int yawHkRow0=6);
	void Init(const CSINS &sins0, int grade=-1);
	virtual void SetFt(int nnq);
	virtual void SetHk(int nnq);
	virtual void Feedback(int nnq, double fbts);
	virtual void SetMeas(void) {};
	void SetMeasGNSS(const CVect3 &posgnss=O31, const CVect3 &vngnss=O31, double yawgnss=0.0);
	void MeasGNSSZvStop(CVect3 &dvnth, double stop=5.0);
	void MeasGNSSZpStop(CVect3 &dposth, double stop=5.0);
	void MeasGNSSZp2X(CVect3 &dposth);
	void Leveling(void);
	int Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts, int nSteps=5); 
#ifdef PSINS_IO_FILE
	void operator<<(CFileRdWt &f);
	void LogXk(void);
#endif
};

class CSysClbt:public CSINSTDKF
{
public:
	int ka2ORpn, iter;
	double wibStatic;
	CVect3 att0, pos0, gn;

	CSysClbt(const CVect3 &pos0, double g00=0.0, int ka2pn=1);
	void Init(double g00=G0);
	void NextIter(const CQuat &qnb0);
	virtual void SetFt(int nnq);
	virtual void SetHk(int nnq) {};
	virtual void SetMeas(void) {};
	virtual void Feedback(int nnq, double fbts);
	int Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts, int isStatic=-1); 
#ifdef PSINS_IO_FILE
	void Log(void);
	void LogXk(void);
#endif
};

#ifdef PSINS_IO_FILE
#ifdef PSINS_RMEMORY
class CPOS618:public CSINSGNSS	// POS618 v.s. AV610
{
	typedef struct { CVect3 wm, vm; double t; CVect3 vngnss, posgnss; double dt; } ImuGnssData;
	typedef struct { CVect3 att, vn, pos, Patt, Pvn, Ppos;	double t; } FXPT; // fusion Xk&Pk&t struct

public:
	double ts;
	int frq, records, iter;
	CVect3 posgnss0;
	ImuGnssData *pIG;
	FXPT *pXP, *pXP1;
	CRMemory *pmemImuGnss, *pmemFusion, *pmemFusion1;
	CSmooth *psmth;
	CFileRdWt *fins, *fkf;

	CPOS618(void);
	CPOS618(double ts, double smthSec=10.0, BOOL isdebug=0);
	~CPOS618();
	BOOL Load(char *fname, double t0=0.0, double t1=0.0);
	void Init(double talign=100.0, const CVect3 &att0=O31);
	void Forward(void);  // forward
	void Backward(void);  // backward & fusion
	void Reverse(void);
	void Smooth(void);
	void Process(char *fname=NULL, int step=0);
};
#endif  // PSINS_RMEMORY
#endif  // PSINS_IO_FILE

class CSINSGNSSCNS:public CSINSGNSS	// SINS/GNSS/CNS
{
public:
	CMat3 Cbs;
	CCNS cns;
	CSINSGNSSCNS(void);
	CSINSGNSSCNS(double ts);
	void SetCNS(int year, int month, int day, double s0=0.0, double dUT1=-0.5, double dTAI=37);
	void Init(const CSINS &sins0, int grade=-1);
	virtual void SetHk(int nnq);
	virtual void Feedback(int nnq, double fbts);
	void SetMeasCNS(CQuat &qCis);
	void SetMeasCNS(CVect3 &vqis);
};

class CAlignkf:public CSINSGNSS
{
public:
	CVect3 pos0;
	CQuat qnb;

	CAlignkf(void);
	CAlignkf(double ts);
	CAlignkf(const CSINS &sins0, double ts);
	void Init(const CSINS &sins0);
	int Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts, int nSteps=5);
	int Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts, const CVect3 &vnr, int nSteps=5);
};

class CAligntrkang:public CSINSGNSS  // coarse alignment by GNSS velocity track-angle
{
public:
	int cntYawOK;
	double velPre, yawPre, vel0, wz0, dyaw0;
	CQuat qnb;

	CAligntrkang(double ts, double vel00=1.0, double wz00=5.0*DPS, double dyaw00=5.0*DEG);
	void Init(const CSINS &sins0=CSINS(O31,O31,O31));
	int Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts, const CVect3 &vnr, int nSteps=5);
};

#ifdef PSINS_RMEMORY
class CAlignsv:public CAlignkf  // initial alignment by data save technique
{
public:
	double t, tk, ts, T1, T2;
	BOOL alnkfinit;
	CAligni0 alni0;
	CRMemory *pMem;

	CAlignsv(void);
	CAlignsv(double ts);
	~CAlignsv();
	CAlignsv(const CVect3 &pos, double ts, double T2=300.0, double T1=0.0);
	void Init(const CVect3 &pos, double ts, double T2=300.0, double T1=0.0);
	int Update(const CVect3 *pwm, const CVect3 *pvm, int nSteps=5);
};
#endif // PSINS_RMEMORY for CAlignrv

class CAligntf:public CSINSGNSS  // transfer alignment by 'velocity+attitude' method
{
public:
	CVect3 mu, lvMINS;
	double dtMINSdelay;
	CAligntf(double ts);
	CAligntf(const CSINS &sins0, double ts);
	void Init(const CSINS &sins0);
	virtual void SetFt(int nnq);
	virtual void SetHk(int nnq);
	virtual void Feedback(int nnq, double fbts);
	int Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts, int nSteps=5);
	void SetMeasVnAtt(const CVect3 &vnMINS=O31, const CVect3 &attMINS=O31);
#ifdef PSINS_IO_FILE
	void operator<<(CFileRdWt &f);
#endif
};

class CSINSGNSSDR:public CSINSGNSS
{
public:
	CVect3 lvOD, posDR, posDRL;
	CMat3 Cbo;			// Cbo: from body-frame to OD-frame
	double Kod;

	CSINSGNSSDR(void);
	CSINSGNSSDR(double ts);
	void Init(const CSINS &sins0, int grade=-1);
	virtual void SetFt(int nnq);
	virtual void SetHk(int nnq) {};
	virtual void Feedback(int nnq, double fbts);
	virtual void SetMeas(void);
	void SetMeasGNSS(const CVect3 &pgnss=O31, const CVect3 &vgnss=O31, double yawgnss=0.0);
	int Update(const CVect3 *pwm, const CVect3 *pvm, double dS, int nSamples, double ts, int nSteps=5); 
};

class CSINSGNSSOD:public CSINSGNSS
{
#define distN 20
public:
	CVect3 IVno, IVni, IVni_1, Ifn, lvOD, vnOD, posOD, odZk;
	CMat3 Cbo, IMv, odMphi, odMvn, odMkappa, odMlever;		// Cbo: from body-frame to OD-frame
	double Kod, Kod2, AOS, odmeast, odmeasT, dS0, afaIVni, distance, distances[distN], distT01, odVel, Hkv2y;
	BOOL odmeasOK;
	int distK0, badODCnt, IVniFirst;

	CSINSGNSSOD(void);
	CSINSGNSSOD(int nq0, int nr0, double ts, int yawHkRow0=10);
	void Init(const CSINS &sins0, int grade=-1);
	void SetDistance(double dist=0.0);
	CVect3 ODKappa(const CVect3 &kpp=O31);
	virtual void SetMeas(void) {};
	BOOL ODVelUpdate(double dS);
	int Update(const CVect3 *pwm, const CVect3 *pvm, double dS, int nSamples, double ts, int nSteps=5); 
#ifdef PSINS_IO_FILE
	void operator<<(CFileRdWt &f);
#endif
};

class CAutoDrive:public CSINSGNSSOD	// automatic drive low-accuracy SINS/GNSS/OD
{
public:
	double *pPkPhiu, *pPkVu, *pPkHgt, *odLost, *zuptLost, gnssYawRMS, gnssLostdist, gnssLostnofixdist;
	double nofixYaw0;
	CWzhold wzhd;
	int satNum, fixMode, fixLast, fixLost, nofixLost, nofixLast, gnssLast;
	CVect3 gnssDOP, posStd; 

	CAutoDrive(void);
	CAutoDrive(double ts);
	void Init(const CSINS &sins0, int grade=-1);
	virtual void SetFt(int nnq);
	virtual void SetHk(int nnq);
	virtual void Feedback(int nnq, double fbts);
	virtual void SetMeas(void) {};
	void ZUPTtest(void);
	void ZIHRtest(void);
	void NHCtest(void);
	void SetGNSSFixMode(int mode=0);
	int Update(const CVect3 *pwm, const CVect3 *pvm, double dS, int nSamples, double ts, int nSteps=5); 
};

class CSGOClbt:public CSINSGNSSOD	// SIMU/GNSS/OD Calibration
{
public:
	CSGOClbt(double ts);
	void Init(const CSINS &sins0, int grade=-1);
	virtual void SetFt(int nnq);
	virtual void SetHk(int nnq);
	virtual void Feedback(int nnq, double fbts);
	int Update(const CVect3 *pwm, const CVect3 *pvm, double dS, int nSamples, double ts, int nSteps=5); 
};

class CVAutoPOS:public CSINSGNSSOD  // high-accuracy Vehicluar Autonomous Positioning & Orientation System, i.e. SINS/OD
{
public:
	CVAutoPOS(void);
	CVAutoPOS(double ts);
	virtual void Init(const CSINS &sins0, int grade=-1);
	virtual void SetFt(int nnq);
	virtual void SetHk(int nnq);
	virtual void Feedback(int nnq, double fbts);
	int Update(const CVect3 *pwm, const CVect3 *pvm, double dS, int nn, double ts, int nStep=5);
};

class CCAM  // Constant Acceleration Model, afa-beta-gamma KF
{
public:
	CVect3 Xk, Qt, Pmin;
	double Rpk, Rvk;
	CMat3 Phi, Pk;
	CCAM(void) {};
	void Init(const CVect3 &pva, const CVect3 &qt, double rp, double rv=0.01, const CVect3 &pva0=O31);
	void TUpdate(double ts, double an=0.0);
	void MUpdate(double Zpk, double Zvk=0.0);  // meas update only
	void Update(double ts, double an, double Zpk, double Zvk=0.0);
};


class CCALLH  // constant acceleration model for lat/lon/hgt
{
public:
	double tk;
	CCAM lat, lon, hgt;
	CCALLH(void) {};
	void Init(const CVect3 &pva, const CVect3 &qt, double rp, double rv=0.01, const CVect3 &pva0=O31);
	void Update(double ts);
	void Update(const CVect3 &vn);
	void Update(double ts, const CVect3 &vn);
	CVect3 GetdPos(const CSINS &sins);
	CVect3 GetdVn(void);
	CVect3 GetPhi(double *dbU=NULL);
	double GetAVP(CVect3 &att, CVect3 &vn, CVect3 &pos, const CSINS &sins);
};

/*
class CCALLH  // constant acceleration model for lat/lon/hgt
{
public:
	double tk;
	CVect3 db, vn, pos, Pdb, Pvn, Ppos, lever;
	CCAM lat, lon, hgt;
	CCALLH(void) {};
	void Init(CSINS &sins, const CVect3 &lv=O31);
	void Update(const CSINS &sins, const CVect3 &posGPS=O31, const CVect3 &vnGPS=O31);
	void Update(const CVect3 &posGPS=O31, const CVect3 &vnGPS=O31);  // meas update only
	void OutLLH(void);
};
*/

class CGKP
{
    double a, b, e2, ep2, cp[6], cn[6];
    CVect3 GKPCore(const CVect3 &BL);
    CVect3 IGKPCore(const CVect3 &xy);
public:
    CGKP(double a0=RE, double f0=FF);
    void Init(double a0=RE, double f0=FF);
    CVect3 GKP(const CVect3 &BL);
    CVect3 IGKP(const CVect3 &xy);
};

#ifdef PSINS_AHRS_MEMS

class CMahony
{
public:
	double tk, Kp, Ki, tau, tau1, dtau;
	CQuat qnb;
	CMat3 Cnb;
	CVect3 exyzInt, ebMax;

	CMahony(double tau=4.0, const CQuat &qnb0=qI);
	void SetTau(double tau=4.0);
	void SetTau(double tau0, double tau1, double dtau=-1.0);
	void SetWn(double wn=1/4.0, double xi=0.707);
	void Update(const CVect3 &wm, const CVect3 &vm, double ts, const CVect3 &mag=O31);
};

class CQEAHRS:public CKalman
{
public:
	CMat3 Cnb;

	CQEAHRS(double ts);
	void Update(const CVect3 &gyro, const CVect3 &acc, const CVect3 &mag, double ts);
};

class CVGHook {
public:
	CMahony mhny;
	CVect3 vn;
	CRAvar RVG;
	CSINSGNSS *pkf;
	int kfidx, isEnable;
	double tk, fasttau, tau, overwbt, maxOverwbt, maxGyro, maxAcc, dkg, dka;
	CVGHook(void);
	void Init(double tau0=2.0, double maxGyro0=600.0, double maxAcc0=10.0);
	void SetHook(CSINSGNSS *kf, int idx=9);
	void Enable(BOOL enable=1);
	int Update(CVect3 &wm, CVect3 &vm, double ts);
};

#endif // PSINS_AHRS_MEMS

#ifdef PSINS_IO_FILE

class CFileRdWt
{
public:
	static char dirIn[256], dirOut[256];
	FILE *rwf;
	char fname[256], line[512], sstr[64*4];
	double buff[64];
	float buff32[64];
	int columns, linek, items[3];
	long totsize, remsize, svpos[3];

	static void DirI(const char *dirI);
	static void DirO(const char *dirO);
	static void Dir(const char *dirI, const char *dirO=(const char*)NULL);
	CFileRdWt(void);
	CFileRdWt(const char *fname0, int columns0=0);
	~CFileRdWt();
	void Init(const char *fname0, int columns0=0);
	int load(int lines=1, BOOL txtDelComma=1);
	int loadf32(int lines=1);
	long load(BYTE *buf, long bufsize);
	void bwseek(int lines, int mod=SEEK_CUR);
	long filesize(int opt=1);
	int getl(void);  // get a line
	long savepos(int i=0);
	int restorepos(int i=0);
	BOOL waitfor(int columnk, double val=0.0, double eps=EPS);
	CFileRdWt& operator<<(double d);
	CFileRdWt& operator<<(const CVect3 &v);
	CFileRdWt& operator<<(const CQuat &q);
	CFileRdWt& operator<<(const CMat3 &m);
	CFileRdWt& operator<<(const CVect &v);
	CFileRdWt& operator<<(const CMat &m);
	CFileRdWt& operator<<(const CRAvar &R);
	CFileRdWt& operator<<(const CMaxMinn &mm);
	CFileRdWt& operator<<(const CPolyfit &pfit);
	CFileRdWt& operator<<(const CPolyfit3 &pfit);
	CFileRdWt& operator<<(const CAlignsb &aln);
	CFileRdWt& operator<<(const CAligni0 &aln);
	CFileRdWt& operator<<(const CIMU &imu);
	CFileRdWt& operator<<(const CSINS &sins);
	CFileRdWt& operator<<(const CDR &dr);
#ifdef PSINS_RMEMORY
	CFileRdWt& operator<<(const CRMemory &m);
#endif
#ifdef PSINS_AHRS_MEMS
	CFileRdWt& operator<<(const CMahony &ahrs);
	CFileRdWt& operator<<(const CQEAHRS &ahrs);
	CFileRdWt& operator<<(const CVGHook &vg);
#endif
#ifdef PSINS_CONSOLE_UART
	CFileRdWt& operator<<(const CUartPP &uart);
#endif
	CFileRdWt& operator<<(CKalman &kf);
	CFileRdWt& operator>>(double &d);
	CFileRdWt& operator>>(CVect3 &v);
	CFileRdWt& operator>>(CQuat &q);
	CFileRdWt& operator>>(CMat3 &m);
	CFileRdWt& operator>>(CVect &v);
	CFileRdWt& operator>>(CMat &m);
};

class CFImuGnssSync
{
public:
	FILE *fimu, *fgps;
	float imuBuf32[32];
	double imuBuf[32], gpsBuf[32], gpsBufNext[32], timu, ts, dT, *tgps, *tgpsNext;
	int imuLen, gpsLen, res;
	CFImuGnssSync(const char *imu, int imuLen, double ts, const char *gps, int gpsLen);
	~CFImuGnssSync();
	int load(int i=1);
};

#ifdef PSINS_IO_FILE_FIND

class CFileList:public CFileRdWt  // for batch file processing
{
public:
	_finddata_t file;
	long handle;
	FILE *flistout;
	int iNextFile, isLastf, iSubfix;
	char fins[32], fkff[32], fxxx[32], fyyy[32], fzzz[32];  // output file names
	const char *lastf;

//	CFileList();
//	CFileList(fname0);
//	CFileList(fname0, listout);
//	CFileList(fname0, firstf, lastf);
//	CFileList(fname0, listout, firstf, lastf);
	CFileList(const char *fname0="filelist.txt", const char *listout=NULL, const char *firstf=NULL, const char *lastf=NULL);
	~CFileList();
	char* NextFile(void);
};

#endif // PSINS_IO_FILE_FIND

class CFileLog
{
public:
	FILE *f, *f0;
	int nLeft, CorMArray;
	clock_t tms0;
	CFileLog(void);
	~CFileLog();
	CFileLog& LogSet(BOOL isLog=1, const char *fname="psinslog.txt");
	CFileLog& LogDate(BOOL hmsOnly=0);
	CFileLog& LogRunTime(BOOL abst=1);
	CFileLog& operator<<(const char *str);
	CFileLog& operator<<(int i);
	CFileLog& operator<<(float f);
	CFileLog& operator<<(double d);
	CFileLog& operator<<(const CVect3 &v);
	CFileLog& operator<<(const CQuat &q);
	CFileLog& operator<<(const CVect &v);
	CFileLog& operator<<(const CMat3 &m);
	CFileLog& operator<<(const CMat &m);
	CFileLog& CArray(const char* varname, const double *var, int row, int clm=1, double scale=1.0, const char* comment=NULL);  // C double array
	CFileLog& MArray(const char* varname, const double *var, int row, int clm=1, double scale=1.0, const char* comment=NULL);  // Matlab double array
	CFileLog& CMArray(const char* varname, const double *var, int row, int clm=1, double scale=1.0, const char* comment=NULL);  // Matlab double array
	void Flush(int n=1000);
};

class CFileCfg
{
#define CfgHdrLen 32
	int isHdr;
public:
	FILE *f;
	CFileCfg(void);
	~CFileCfg();
	CFileCfg& operator<<(const char *hdr);
	CFileCfg& operator>>(const char *hdr);
	CFileCfg& operator<<(short s);
	CFileCfg& operator<<(int i);
	CFileCfg& operator<<(float ff);
	CFileCfg& operator<<(double d);
	CFileCfg& operator<<(const CVect3 &v);
	CFileCfg& operator<<(const CQuat &q);
	CFileCfg& operator<<(const CMat3 &m);
	CFileCfg& operator<<(const CMat &m);
	CFileCfg& operator>>(short &s);
	CFileCfg& operator>>(int &i);
	CFileCfg& operator>>(float &ff);
	CFileCfg& operator>>(double &d);
	CFileCfg& operator>>(CVect3 &v);
	CFileCfg& operator>>(CQuat &q);
	CFileCfg& operator>>(CMat3 &m);
	CFileCfg& operator=(CFileCfg &cfg);
};
CFileCfg WriteCfg(const char *fname, const char *ext=NULL);  // the only method to create WriteCfg file
CFileCfg ReadCfg(const char *fname, const char *ext=NULL);   // the only method to create ReadCfg file

#endif // PSINS_IO_FILE

#ifdef PSINS_RMEMORY

#define MAX_RECORD_BYTES 512

class CRMemory
{
public:
	BYTE *pMemStart0, *pMemStart, *pMemEnd;
	int pushLen, popLen, recordNum, recordLen;
	long memLen, dataLen;
	BYTE *pMemPush, *pMemPop, pushBuf[MAX_RECORD_BYTES], popBuf[MAX_RECORD_BYTES];

	CRMemory(void);
	CRMemory(long recordNum, int recordLen0);
	CRMemory(BYTE *pMem, long memLen0, int recordLen0=0);
	~CRMemory();
	void Init(BYTE *pMem, long memLen0, int recordLen0);
	void operator=(const CRMemory &m);
	BOOL push(const BYTE *p=(const BYTE*)NULL);
	BYTE pop(BYTE *p=(BYTE*)NULL);
	BYTE* get(int iframe);
	BYTE* set(int iframe, const BYTE *p);
};

class CSmooth
{
public:
	CRMemory *pmem;
	CVect sum, mean, tmp;
	int irow, maxrow;
	CSmooth(int clm=MMD, int row=100);
	~CSmooth();
	CVect Update(const double *p, double *pmean=NULL);
};

class CInterp		// linear interpolation
{
public:
	CRMemory *pmem, mem;
	int irow, row, clm;
	double mint, maxt;
	CInterp(double **table, int row, int clm=2);
	CInterp(const char *fname, int clm=2);
	~CInterp();
	double Interp(double t, double *data=NULL);
};

#endif // PSINS_RMEMORY

class CContinuousCnt  // continuous count
{
	int cnt0, cntLargest0, isFirst, lost;
public:
	CContinuousCnt(int cntLargest=255);
	int Update(int cnt);
};

class CUartPP
{
public:
#define UARTFRMLEN  (512)
#define UARTBUFLEN  (UARTFRMLEN*100)
	unsigned char head[2], popbuf[UARTFRMLEN], buf[UARTBUFLEN], chksum;
	int pushIdx, popIdx, frameLen, overflow, getframe, safeBytes;
	int csflag, cs0, cs1, css;   // 0: no checksum, 1: uchar sum, 2: crc8; popbuf[css]==popbuf[cs0]+...+popbuf[cs1] 
//unsigned short chksum;

	CUartPP(int frameLen0=10, unsigned short head0=0x55aa);  // NOTE: char {0xaa 0x55 ...} for little-endian
	BOOL checksum(const unsigned char *pc);
	inline int nextIdx(int idx);
	int push(const unsigned char *buf0, int len);
	int pop(unsigned char *buf0=(unsigned char*)NULL);
#ifdef PSINS_IO_FILE
	BOOL safeWrite(CFileRdWt &f, int safeBytes0=(1<<20));
#endif
};
unsigned char chksum8(const unsigned char *pc, int len);
unsigned short chksum16(const unsigned short *ps, int len);
int makefrm(void *buf, ...);
double flt22db(float f1, float f2);
void db2flt2(double db, float *pf1, float *pf2);
int iga2flt(float *pf, const double *pwm, const double *pvm=NULL, const double *pgps=NULL, const double *pavp=NULL, const double *pt=NULL);

#ifdef PSINS_CONSOLE_UART

typedef struct {
	float hd, t, gx, gy, gz, ax, ay, az, magx, magy, magz, bar, pch, rll, yaw, ve, vn, vu,
		lon0, lon1, lat0, lat1, hgt, gpsve, gpsvn, gpsvu, gpslon0, gpslon1, gpslat0, gpslat1, gpshgt,
		gpsstat, gpsdly, tmp, sum;
} PSINSBoard;

#include "..\uart\serialport.h"

extern "C" WINBASEAPI HWND WINAPI GetConsoleWindow();

extern CUartPP *pUart;

class CConUart {	// console & uart control
public:
	char cmd[2], cmdstr[128], cmdlen;
	int comi;
	HANDLE hConsole;
	CUartPP uart;
	itas109::CSerialPort m_SerialPort;
	CFileRdWt fraw;
	PSINSBoard *ps;
	CConUart(void);
	void Init(int COMi, int BaudRate, int frameLen, unsigned short head);
//	void onExit(void);
	BOOL getUart(void);
	void dispUart(int itv=11);
	void gotorc(SHORT row, SHORT clm=0);
	void sendUart(void);
};

class CComUart {	// common uart control
public:
	struct US { char type; int len; double scale; char name[64]; } uarts[80];
	int comi, sendLen, bSwapbytes;
	HANDLE hConsole;
	CUartPP uart;
	itas109::CSerialPort m_SerialPort;
	CFileRdWt fraw;
	CComUart(void);
	void Init(char *fsetting);
	void Init(int comi, int baudrate, int bSwap=0);
	BOOL getUart(void);
	void dispUart(int itv=11);
	void sendUart(uchar *buf, int len, int waitms=0);
	void gotorc(SHORT row, SHORT clm=0);
};

#endif // PSINS_CONSOLE_UART

class CBytemani {
public:
};
unsigned short swap16(unsigned short ui16);
unsigned int swap32(unsigned int ui32);
unsigned long swap64(unsigned long ui64);
unsigned char* int24(unsigned char *pchar3, int int32);
unsigned char* swap24(unsigned char *puc3, unsigned char *pres=NULL);
unsigned char* swapbytes(unsigned char *pc, int nBytes);
int diffint24(const unsigned char *pc1, const unsigned char *pc0);
int i24i32(const unsigned char *pchar3);  // int24 to int32

#ifdef PSINS_VC_AFX_HEADER

#include "afx.h"
class CVCFileFind {		// for file operation in the same folder
public:
	BOOL lastfile;
	char fname[256];
	CFileFind finder;
	CVCFileFind(char *path, char *filetype);
	char* FindNextFile(void);
};

#endif // PSINS_VC_AFX_HEADER

#ifdef PSINS_EXTERN_C_EXAMPLE

// an example called by C-main file, and copy the defines to a C-header file
#ifdef __cplusplus
extern "C" {
#endif
void psinsInit0(const double *att0, const double *vn0, const double *pos0, double t0);
void psinsUpdate0(const double *gyro, const double *acc, int n, double ts); // IMU double type
void psinsUpdatef(const float *gyro, const float *acc, int n, double ts);	// IMU float type
void psinsOut0(double *att, double *vn, double *pos, double *t);
#ifdef __cplusplus
};
#endif

#endif // PSINS_EXTERN_C_EXAMPLE

void add(CVect3 &res, const CVect3 &v1, const CVect3 &v2);
void add(CMat3 &res, const CMat3 &m1, const CMat3 &m2);
void add(CVect &res, const CVect &v1, const CVect &v2);
void add(CMat &res, const CMat &m1, const CMat &m2);
void sub(CVect3 &res, const CVect3 &v1, const CVect3 &v2);
void sub(CMat3 &res, const CMat3 &m1, const CMat3 &m2);
void cros(CVect3 &res, const CVect3 &v1, const CVect3 &v2);
void dotmul(CVect3 &res, const CVect3 &v1, const CVect3 &v2);
void dotdiv(CVect3 &res, const CVect3 &v1, const CVect3 &v2);
void mul(CVect3 &res, const CVect3 &v, const double &f);
void mul(CMat3 &res, const CMat3 &m, const double &f);
void mul(CVect &res, const CVect &v, const double &f);
void mul(CVect &res, const CMat &m, const CVect &v);
void mul(CMat &res, const CMat &m, const double &f);
void mul(CVect3 &res, const CMat3 &m, const CVect3 &v);
void mul(CVect3 &res, const CVect3 &v, const CMat3 &m);
void mul(CMat3 &res, const CMat3 &m1, const CMat3 &m2);
void mul(CQuat &res, const CQuat &q1, const CQuat &q2);
void mul(CVect3 &res, const CQuat &q, const CVect3 &v);
void _TT(CMat3 &mT, const CMat3 &m); 
void rv2q(CQuat &q, const CVect3 &rv);
void qdelphi(CQuat &q, const CVect3 &phi);
void q2mat(CMat3 &Cnb, const CQuat &qnb);
void m2att(CVect3 &att, const CMat3 &Cnb);
void AXbt(CVect3 &res, const CMat3 &A, const CVect3 &X, const CVect3 &b, const double &t=1.0);  // res=A*X1+X2*w

#pragma pack()

#endif // _PSINS_H
