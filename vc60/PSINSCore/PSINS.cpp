
/* PSINS(Precise Strapdown Inertial Navigation System) C++ algorithm source file PSINS.cpp

Copyright(c) 2015-2023, by YanGongmin, All rights reserved.
Northwestern Polytechnical University, Xi'an, P.R.China.
Date: 17/02/2015, 19/07/2017, 11/12/2018, 27/12/2019, 12/12/2020, 22/11/2021, 17/10/2022, 23/08/2023
      16/06/2024
*/

#include "PSINS.h"

const CVect3 O31(0.0), One31(1.0), I31Z(0,0,1.0), Ipos(1.0/RE,1.0/RE,1.0), posNWPU=LLH(34.034310, 108.775427, 450); //NWPU-Lab-pos
const CQuat  qI(1.0,0.0,0.0,0.0);
const CMat3  I33(1,0,0, 0,1,0, 0,0,1), O33(0,0,0, 0,0,0, 0,0,0), One33(1.0);
const CVect  On1(MMD,0.0), O1n=~On1, Onen1(MMD,1.0);
CVect3       Vrbs(0.0);
CGLV		 glv;
#ifdef PSINS_IO_FILE
CFileLog	 psinslog;
CFileRdWt	 *pfDebug=NULL;
#endif
int			 psinslasterror = 0;
int			 psinsstack0 = 0, psinsstacksize = 0;

//***************************  class CGLV  *********************************/
CGLV::CGLV(double Re, double f, double g0)
{
	this->Re = Re; this->f = f; this->g0 = g0; this->wie = WIE;
	Rp = (1-f)*Re;
	e = sqrt(2*f-f*f); e2 = e*e;
	ep = sqrt(Re*Re-Rp*Rp)/Rp; ep2 = ep*ep;
    mg = g0/1000.0;
    ug = mg/1000.0;
    deg = PI/180.0;
    min = deg/60.0;
    sec = min/60.0;
    ppm = 1.0e-6;
    hur = 3600.0;
	dps = deg/1.0;
    dph = deg/hur;
    dpsh = deg/sqrt(hur);
    dphpsh = dph/sqrt(hur);
    dph2 = dph/hur;
	dphpg = dph/g0;
    ugpsHz = ug/sqrt(1.0);
    ugpsh = ug/sqrt(hur);
	ugpg2 = ug/g0/g0;
    mpsh = 1/sqrt(hur); 
    mpspsh = 1/1/sqrt(hur);
    ppmpsh = ppm/sqrt(hur);
    secpsh = sec/sqrt(hur);
#ifdef PSINS_IO_FILE
	t0 = clock();
#endif
}

#ifdef PSINS_IO_FILE
int CGLV::toc(BOOL disp)
{
	clock_t t1 = clock();
	int res = t1-t0;  t0 = t1;
	if(disp) printf("\n\tPSINS elasped time (ms): %d.\n\n", res);
	return res;
}

CGLV::~CGLV(void)
{
	toc(1);
}
#endif

#ifdef PSINS_COMPLEX
//***************************  class CComplex  *********************************/
CComplex::CComplex(double a0, double b0)
{
	a=a0, b=b0;
}

CComplex CComplex::operator+(const CComplex &z) const
{
	return CComplex(a+z.a, b+z.b);
}

CComplex CComplex::operator+(double a0) const
{
	return CComplex(a+a0, b);
}

CComplex operator+(double a0, const CComplex &z)
{
	return CComplex(a0+z.a, z.b);
}

CComplex CComplex::operator-(const CComplex &z) const
{
	return CComplex(a-z.a, b-z.b);
}

CComplex CComplex::operator-(double a0) const
{
	return CComplex(a-a0, b);
}

CComplex operator-(double a0, const CComplex &z)
{
	return CComplex(a0-z.a, -z.b);
}

CComplex CComplex::operator*(const CComplex &z) const
{
	return CComplex(a*z.a-b*z.b, a*z.b+b*z.a);
}

CComplex CComplex::operator*(double a0) const
{
	return CComplex(a*a0, b*a0);
}

CComplex operator*(double a0, const CComplex &z)
{
	return CComplex(a0*z.a, a0*z.b);
}

CComplex CComplex::operator/(const CComplex &z) const
{
	double n=z.a*z.a+z.b*z.b;
	if(n>0.0) {
		CComplex z1(z.a/n, -z.b/n);
		return *this*z1;
	}
	else {
		return CComplex(INF);
	}
}

CComplex CComplex::operator/(double a0) const
{
	if(a==0.0) return CComplex(INF);
	else return CComplex(a/a0, b/a0);
}

CComplex operator/(double a0, const CComplex &z)
{
	return CComplex(a0)/z;
}

CComplex operator-(const CComplex &z)
{
	return CComplex(-z.a, -z.b);
}

CComplex& CComplex::operator=(double a0)
{
	a = a0, b = 0.0;
	return *this;
}

CComplex operator~(const CComplex &z)
{
	return CComplex(z.a, -z.b);
}

double real(const CComplex &z)
{
	return z.a;
}

double img(const CComplex &z)
{
	return z.b;
}

double norm(const CComplex &z)
{
	return sqrt(z.a*z.a+z.b*z.b);
}

double arg(const CComplex &z)
{
	double n = z.a*z.a+z.b*z.b;
	if(n==0.0) return 0;
	else return atan2(z.b, z.a);
}

CComplex pow(const CComplex &z, double k)
{
	if(k==0.0) return CComplex(1.0);
	double n=pow(z.a*z.a+z.b*z.b,k/2), Arg=arg(z)*k;
	return CComplex(n*cos(Arg), n*sin(Arg));
}

CComplex sqrt(const CComplex &z)
{
	double n=sqrt(sqrt(z.a*z.a+z.b*z.b)), Arg=arg(z)/2.0;
	return CComplex(n*cos(Arg), n*sin(Arg));
}

CVect3 m33abc(const CMat3 &m)
{
	CMat3 B=MMT(m);
	double e012=B.e01*B.e01, e022=B.e02*B.e02, e122=B.e12*B.e12, a, b, c;
	a = -(B.e00+B.e11+B.e22);
	b = B.e00*B.e11+B.e00*B.e22+B.e11*B.e22-e012-e022-e122;
	c = B.e00*e122+B.e11*e022+B.e22*e012-B.e00*B.e11*B.e22-2*B.e01*B.e02*B.e12;
	return CVect3(a,b,c);
}

CVect3 realrt3(double a, double b, double c)
{
	double a3=a/3, p=b-a*a3, q=2.0/27*a*a*a-a3*b+c; // p=(3*b-a*a)/3, q=(2*a*a*a-9*a*b+27*c)/27;
	CComplex x1, x2, x3;
	if(p==0.0 && q>=0.0) {
		x1 = x2 = x3 = -pow(q,1.0/3);
	}
	else {
		double Delta=4*p*p*p+27*q*q;
		CComplex Delta1=pow(sqrt(3*CComplex(Delta))/18.0-q/2.0,1.0/3);
		CComplex rp60(0.5,sqrt3/2.0), rn60(0.5,-sqrt3/2.0),
			p3D=p/3/Delta1;
		x1 =    -(p3D -      Delta1),
		x2 = rp60*p3D - rn60*Delta1,
		x3 = rn60*p3D - rp60*Delta1;
	}
	return sort(CVect3(x1.a-a3, x2.a-a3, x3.a-a3));
}

CVect3 ShengJin(double a, double b, double c)
{
	double a3=a/3, p=b-a*a3, q=2.0/27*a*a*a-a3*b+c; // p=(3*b-a*a)/3, q=(2*a*a*a-9*a*b+27*c)/27;
	double n3p, x, theta3, x1, x2, x3;
	if(p>-EPS) p=-EPS;
	n3p = sqrt(-3*p);
	x = 4.5*q/p/n3p;
	if(x>1.0) x=1.0; else if(x<-1.0) x=-1.0;
	theta3 = acos(x)/3; 
	n3p = 2.0/3*n3p;
	x1 = n3p*cos(theta3);
	x2 = n3p*cos(theta3+_2PI/3); 
	x3 = n3p*cos(theta3-_2PI/3); 
	return sort(CVect3(x1-a3, x2-a3, x3-a3));
}

#endif  //PSINS_COMPLEX

//***************************  class CVect3  *********************************/
CVect3::CVect3(void)
{
}

CVect3::CVect3(double xyz)
{
	i=j=k=xyz;
}

CVect3::CVect3(double xx, double yy, double zz)
{
	i=xx, j=yy, k=zz;
}

CVect3::CVect3(const double *pdata)
{
	i=*pdata++, j=*pdata++, k=*pdata;
}

CVect3::CVect3(const float *pdata)
{
	i=*pdata++, j=*pdata++, k=*pdata;
}

CVect3::CVect3(const int *pdata, double f)
{
	i=f**pdata++, j=f**pdata++, k=f**pdata;
}

BOOL IsZero(const CVect3 &v, double eps)
{
	return (v.i<eps&&v.i>-eps && v.j<eps&&v.j>-eps && v.k<eps&&v.k>-eps);
}

BOOL IsZeroXY(const CVect3 &v, double eps)
{
	return (v.i<eps&&v.i>-eps && v.j<eps&&v.j>-eps);
}

BOOL IsNaN(const CVect3 &v)
{
	return 0; //(_isnan(i) || _isnan(j) || _isnan(k));
}

BOOL IsRad(const CVect3 &v, int n)
{
	const double *p=&v.i;
	for(int i=0; i<n; i++,p++) {
		if(*p<-PI-EPS||*p>_2PI+EPS) return FALSE;
	}
	return TRUE;
}

double AccScale(const CVect3 &mfb, double ts, double g)
{
	return (g*ts)/norm(mfb);  // mfb: static mean acc sampling
}

CVect3 CVect3::operator+(const CVect3 &v) const
{
	return CVect3(this->i+v.i, this->j+v.j, this->k+v.k);
}

CVect3 CVect3::operator-(const CVect3 &v) const
{
	return CVect3(this->i-v.i, this->j-v.j, this->k-v.k);
}

CVect3 CVect3::operator*(const CVect3 &v) const
{
	stacksize();
	return CVect3(this->j*v.k-this->k*v.j, this->k*v.i-this->i*v.k, this->i*v.j-this->j*v.i);
}
	
CVect3 CVect3::operator*(double f) const
{
	return CVect3(i*f, j*f, k*f);
}

CVect3 CVect3::operator*(const CMat3 &m) const
{
	return CVect3(i*m.e00+j*m.e10+k*m.e20,i*m.e01+j*m.e11+k*m.e21,i*m.e02+j*m.e12+k*m.e22);
}
	
CVect3 CVect3::operator/(double f) const
{
	return CVect3(i/f, j/f, k/f);
}

CVect3 CVect3::operator/(const CVect3 &v) const
{
	return CVect3(i/v.i, j/v.j, k/v.k);
}

CVect3& CVect3::operator=(double f)
{
	i = j = k = f;
	return *this;
}

CVect3& CVect3::operator=(const double *pf)
{
	i = *pf++, j = *pf++, k = *pf;
	return *this;
}

CVect3& CVect3::operator+=(const CVect3 &v)
{ 
	i += v.i, j += v.j, k += v.k;
	return *this;
}

CVect3& CVect3::operator-=(const CVect3 &v)
{ 
	i -= v.i, j -= v.j, k -= v.k;
	return *this;
}

CVect3& CVect3::operator*=(double f)
{ 
	i *= f, j *= f, k *= f;
	return *this;
}

CVect3& CVect3::operator/=(double f)
{ 
	i /= f, j /= f, k /= f;
	return *this;
}

CVect3& CVect3::operator/=(const CVect3 &v)
{ 
	i /= v.i, j /= v.j, k /= v.k;
	return *this;
}

CVect3 operator*(double f, const CVect3 &v)
{
	return CVect3(v.i*f, v.j*f, v.k*f);
}
	
CVect3 operator-(const CVect3 &v)
{
	return CVect3(-v.i, -v.j, -v.k);
}

double& CVect3::operator()(int r)
{
	return (&this->i)[r];
}

double crossXY(const CVect3 &v1, const CVect3 &v2)
{
	return v1.i*v2.j-v1.j*v2.i;
}

CMat3 vxv(const CVect3 &v1, const CVect3 &v2)
{
	return CMat3(v1.i*v2.i, v1.i*v2.j, v1.i*v2.k, 
				 v1.j*v2.i, v1.j*v2.j, v1.j*v2.k, 
				 v1.k*v2.i, v1.k*v2.j, v1.k*v2.k);
}

CVect3 sqrt(const CVect3 &v)
{
	return CVect3(sqrt(v.i), sqrt(v.j), sqrt(v.k));
}

CVect3 pow(const CVect3 &v, int k)
{
	CVect3 pp = v;
	for(int i=1; i<k; i++)
	{
		pp.i *= v.i, pp.j *= v.j, pp.k *= v.k;
	}
	return pp;
}

CVect3 abs(const CVect3 &v)
{
	CVect3 res;
	res.i = v.i>0.0 ? v.i : -v.i;
	res.j = v.j>0.0 ? v.j : -v.j;
	res.k = v.k>0.0 ? v.k : -v.k;
	return res;
}

inline double absp(double val)
{
	return val>0.0 ? val : 0.0;
}

CVect3 absp(const CVect3 &v)
{
	return CVect3(absp(v.i),absp(v.j),absp(v.k));
}

CVect3 maxabs(const CVect3 &v1, const CVect3 &v2)
{
	CVect3 res1, res2;
	res1.i = v1.i>0.0 ? v1.i : -v1.i;  res2.i = v2.i>0.0 ? v2.i : -v2.i;  if(res1.i<res2.i) res1.i=res2.i;
	res1.j = v1.j>0.0 ? v1.j : -v1.j;  res2.j = v2.j>0.0 ? v2.j : -v2.j;  if(res1.j<res2.j) res1.j=res2.j;
	res1.k = v1.k>0.0 ? v1.k : -v1.k;  res2.k = v2.k>0.0 ? v2.k : -v2.k;  if(res1.k<res2.k) res1.k=res2.k;
	return res1;
}

double norm(const CVect3 &v)
{
	return sqrt(v.i*v.i + v.j*v.j + v.k*v.k);
}

double normlize(CVect3 *v)
{
	double n=norm(*v);
	v->i /= n, v->j /= n, v->k /= n;
	return n;
}

double normInf(const CVect3 &v)
{
	double i = v.i>0 ? v.i : -v.i,
		   j = v.j>0 ? v.j : -v.j,
		   k = v.k>0 ? v.k : -v.k;
	if(i>j)	return i>k ? i : k;
	else    return j>k ? j : k;
}

double normXY(const CVect3 &v)
{
	return sqrt(v.i*v.i + v.j*v.j);
}

double normXYInf(const CVect3 &v)
{
	double i = v.i>0 ? v.i : -v.i,
		   j = v.j>0 ? v.j : -v.j;
	return i>j ? i : j;
}

double dot(const CVect3 &v1, const CVect3 &v2)
{
	return (v1.i*v2.i + v1.j*v2.j + v1.k*v2.k);
}

CVect3 dotmul(const CVect3 &v1, const CVect3 &v2)
{
	return CVect3(v1.i*v2.i, v1.j*v2.j, v1.k*v2.k);
}

CVect3 dotdiv(const CVect3 &v1, const CVect3 &v2)
{
	return CVect3(v1.i/v2.i, v1.j/v2.j, v1.k/v2.k);
}

CQuat rv2q(const CVect3 &rv)
{
#define rvF1	(     2 * 1)		// define: Fk=2^k*k! 
#define rvF2	(rvF1*2 * 2)
#define rvF3	(rvF2*2 * 3)
#define rvF4	(rvF3*2 * 4)
#define rvF5	(rvF4*2 * 5)
	double n2 = rv.i*rv.i+rv.j*rv.j+rv.k*rv.k, c, f;
	if(n2<(PI/180.0*PI/180.0))	// 0.017^2 
	{
		double n4=n2*n2;
		c = 1.0 - n2*(1.0/rvF2) + n4*(1.0/rvF4);
		f = 0.5 - n2*(1.0/rvF3) + n4*(1.0/rvF5);
	}
	else
	{
		double n_2 = sqrt(n2)/2.0;
		c = cos(n_2);
		f = sin(n_2)/n_2*0.5;
	}
	return CQuat(c, f*rv.i, f*rv.j, f*rv.k);
}

CMat3 rv2m(const CVect3 &rv)
{
	return q2mat(rv2q(rv));
}

CMat3 askew(const CVect3 &v)
{
	return CMat3(0,  -v.k, v.j, 
				 v.k, 0.0,  -v.i,
				-v.j, v.i, 0);
}

CMat3 pos2Cen(const CVect3 &pos)
{
	double si = sin(pos.i), ci = cos(pos.i), sj = sin(pos.j), cj = cos(pos.j);
	return CMat3(	-sj, -si*cj,  ci*cj,  
					 cj, -si*sj,  ci*sj,  
					 0,   ci,     si      );	//Cen
}

CVect3 pp2vn(const CVect3 &pos1, const CVect3 &pos0, double ts, CEarth *pEth)
{
	double sl, cl, sl2, sq, sq2, RMh, RNh, clRNh;
	if(pEth)
	{
		RMh = pEth->RMh; clRNh = pEth->clRNh;
	}
	else
	{
		sl=sin(pos0.i); cl=cos(pos0.i); sl2=sl*sl;
		sq = 1-glv.e2*sl2; sq2 = sqrt(sq);
		RMh = glv.Re*(1-glv.e2)/sq/sq2+pos0.k;
		RNh = glv.Re/sq2+pos0.k;    clRNh = cl*RNh;
	}
    CVect3 vn = pos1 - pos0;
    return CVect3(vn.j*clRNh/ts, vn.i*RMh/ts, vn.k/ts);
}

CVect3 pp2att(const CVect3 &pos1, const CVect3 &pos0)
{
	return vn2att(pp2vn(pos1, pos0, 1.0));
}

double sinAng(const CVect3 &v1, const CVect3 &v2, const CVect3 &v0)
{
	if(&v0!=&O31)
		return sinAng(v1-v0,v2-v0);
	if(IsZero(v1)||IsZero(v2)) return 0.0;
	return norm(v1*v2)/(norm(v1)*norm(v2));
}

double MagYaw(const CVect3 &mag, const CVect3 &att, double declination)
{
	CVect3 attH(att.i, att.j, 0.0);
	CVect3 magH = a2mat(attH)*mag;
	double yaw = 0.0;
//	if(attH.i<(80.0*DEG)&&attH.i>-(80.0*DEG))
	{
		yaw = atan2Ex(magH.i, magH.j) + declination;
		if(yaw>PI)       yaw -= _2PI;
		else if(yaw<-PI) yaw += _2PI;
	}
	return yaw;
}

CVect3 xyz2blh(const CVect3 &xyz)
{
	double s = normXY(xyz), theta = atan2(xyz.k*glv.Re, s*glv.Rp),
		s3 = sin(theta), c3 = cos(theta); s3 = s3*s3*s3, c3 = c3*c3*c3;
	if(s<(6378137.0*1.0*DEG))  return O31;
	double L = atan2(xyz.j, xyz.i), B = atan2(xyz.k+glv.ep2*glv.Rp*s3, s-glv.e2*glv.Re*c3),
		sB = sin(B), cB = cos(B), N = glv.Re/sqrt(1-glv.e2*sB*sB);
	return CVect3(B, L, s/cB-N);
}

CVect3 blh2xyz(const CVect3 &blh)
{
	double sB = sin(blh.i), cB = cos(blh.i), sL = sin(blh.j), cL = cos(blh.j),
		N = glv.Re/sqrt(1-glv.e2*sB*sB);
	return CVect3((N+blh.k)*cB*cL, (N+blh.k)*cB*sL, (N*(1-glv.e2)+blh.k)*sB);
}

CVect3 Vxyz2enu(const CVect3 &Vxyz, const CVect3 &pos)
{
	return Vxyz*pos2Cen(pos);
}

CVect3 v2double(double f)
{
	CVect3 v;
	v.i = (int)(f+EPS);	f = (f-v.i)*1.0e8;  if(v.i>50000000.0) v.i -= 100000000.0;  // -45000000 ~ +45000000
	v.j = (int)(f+EPS);						if(v.j>50000000.0) v.j -= 100000000.0;
	v.k = 0.0;
	return v;
}

CVect3 v3double(double f)
{
	CVect3 v;
	v.i = (int)(f+EPS);	f = (f-v.i)*1.0e5;  if(v.i>50000.0) v.i -= 100000.0;  // -45000 ~ +45000
	v.j = (int)(f+EPS);	f = (f-v.j)*1.0e5;  if(v.j>50000.0) v.j -= 100000.0;
	v.k = (int)(f+EPS);						if(v.k>50000.0) v.k -= 100000.0;
	return v;
}

void v5double(double f, CVect3 &v1, CVect3 &v2)
{
	psinsassert(f>0.0);
	v1.i = (int)(f+EPS);	f = (f-v1.i)*1.0e3;   // 0 ~ +990
	v1.j = (int)(f+EPS);	f = (f-v1.j)*1.0e3;
	v1.k = (int)(f+EPS);	f = (f-v1.k)*1.0e3;
	v2.i = (int)(f+EPS);	f = (f-v2.i)*1.0e3;
	v2.j = (int)(f+EPS);
	v2.k = 0.0;
}

void v2flt(float *pf, const CVect3 *pv, ...)  // v2flt(pf, &v1, &v2, ..., NULL);
{
	va_list vl;
	va_start(vl, pv);
	while(1) {
		*pf++=(float)pv->i, *pf++=(float)pv->j, *pf++=(float)pv->k;
		pv = va_arg(vl, CVect3*);
		if(pv==NULL) break;
	}
	va_end(vl);
}

CVect3 sort(const CVect3 &v)
{
	CVect3 vtmp=v;
	if(vtmp.i<vtmp.j) swapt(vtmp.i,vtmp.j,double);
	if(vtmp.i<vtmp.k) swapt(vtmp.i,vtmp.k,double);
	if(vtmp.j<vtmp.k) swapt(vtmp.j,vtmp.k,double);
	return vtmp;
}

double median(const double &f1, const double &f2, const double &f3)
{
	return median(CVect3(f1,f2,f3));
}

double median(const CVect3 &v)
{
	double res;
	if(v.i>v.j) {
		if(v.j>v.k) res=v.j;
		else if(v.k>v.i) res=v.i;
		else res=v.k;
	}
	else { // v.i<=v.j
		if(v.j<v.k) res=v.j;
		else if(v.i>v.k) res=v.i;
		else res=v.k;
	}
	return res;
}

CVect3 dm2r(const CVect3 &v, int typ)
{
	if(typ==2) return CVect3(dm2r(v.i), dm2r(v.j), v.k);
	return CVect3(dm2r(v.i), dm2r(v.j), dm2r(v.k));
}

CVect3 v3mmm(const CVect3 &p1, const CVect3 &p2, const CVect3 &p3, CVect3 *pmm, BOOL isMean, const CVect3 &th)
{
	CVect3 res, mm;
	double *pf0=&res.i, *pm=&mm.i; const double *pf1=&p1.i, *pf2=&p2.i, *pf3=&p3.i;
	for(int i=0; i<3; i++,pf0++,pm++,pf1++,pf2++,pf3++) {
		if(*pf1>*pf2) {
			if(*pf2>*pf3) { *pf0=*pf2, *pm=*pf1-*pf3; }
			else if(*pf3>*pf1) { *pf0=*pf1, *pm=*pf3-*pf1; }
			else { *pf0=*pf3, *pm=*pf1-*pf2; }
		}
		else { // pf1<pf2
			if(*pf2<*pf3) { *pf0=*pf2, *pm=*pf3-*pf1; }
			else if(*pf1>*pf3) { *pf0=*pf1, *pm=*pf2-*pf3; }
			else { *pf0=*pf3, *pm=*pf2-*pf1; } 
		}
	}
	if(pmm) *pmm=mm;  // max-min
	if(isMean) {
		if(mm.i<th.i) res.i=(p1.i+p2.i+p3.i)/3.0;
		if(mm.j<th.j) res.j=(p1.j+p2.j+p3.j)/3.0;
		if(mm.k<th.k) res.k=(p1.k+p2.k+p3.k)/3.0;
	}
	return res;
}

CVect3 fopp(const CVect3 &a, const CVect3 &b, const CVect3 &c)  // point c to line a-b
{
	CVect3 ab=b-a;
	double u=dot(c-a,ab), n2=ab.i*ab.i+ab.j*ab.j+ab.k*ab.k;
	return n2<1.0e-20 ? a : a+ab*u/n2;
}

CVect3 fopp(const CVect3 &a, const CVect3 &b, const CVect3 &c, const CVect3 &d)  // point d to plane a-b-c, https://www.docin.com/p-2324716540.html
{
	CVect3 ABC=-inv(CMat3(a,b,c))*One31, ab=b-a, ac=c-a;
	return inv(CMat3(ab,ac,ABC))*CVect3(dot(d,ab),dot(d,ac),-1);
}

CVect3 tp2att(const CVect3 &a, const CVect3 &b, const CVect3 &c)  // a-b-c in couter-clockwise
{
	CVect3 d=fopp(a, b, c);
	CVect3 att = dv2att(c-d,a-d, CVect3(1,0,0),CVect3(0,1,0));  // Att of c-d-a(x-o-y)
	CVect3 vo = fopp(a, b, c, O31);  // orthocenter
//	CVect3 vo = (a+b+c)/3.0;  // center of gravity
	double afa=asin(sinAng(vo, b, a));
	return q2att(a2qua(att)*a2qua(CVect3(0,0,afa)));  // Att of *-fop-a(x-o-y)
}

CVect3 addw(const CVect3 &X1, const CVect3 &X2, double w1, double w2)
{
	if(w2>INFp5) w2=1.0-w1;
	return CVect3(w1*X1.i+w2*X2.i, w1*X1.j+w2*X2.j, w1*X1.k+w2*X2.k);
}

CVect3 attract(const CVect3 &v, const CVect3 &th, const CVect3 &center)
{
	CVect3 dv=v-center;
	if(dv.i>th.i || dv.i<-th.i || dv.j>th.j || dv.j<-th.j || dv.k>th.k || dv.k<-th.k) return v;
	return CVect3(attract(v.i,th.i,center.i), attract(v.j,th.j,center.j), attract(v.k,th.k,center.k));
}

CVect3 ff2muxy(const CVect3 &f0, const CVect3 &f1, const char *dir0, const char *dir1)
{
    CVect3 v0 = f0, v1 = f1;
    if(dir0) IMURFU(&v0, 1, dir0);  if(dir1) IMURFU(&v1, 1, dir1);
    double n = norm(v0), f = cos(v0.j/n);  if(f<0.1) f=0.1;
    return CVect3((v1.j-v0.j)/f, v0.i-v1.i, 0)/norm(v0);
}

CVect3 ff2mu(const CVect3 &f0, const CVect3 &f1, double uz)
{
	CVect3 df=f1-f0, mu;
	double fz = 7.5*f0.k*f0.k<f0.i*f0.i+f0.j*f0.j ? 10.0 : f0.k;  // 20deg
	mu.i = (uz*f1.i-df.j)/fz;
	mu.j = (df.i+uz*f1.j)/fz;
	mu.k = uz;
	return mu;
}

//***************************  class CQuat  *********************************/
CQuat::CQuat(void)
{
}

CQuat::CQuat(double qq0, const CVect3 &qqv)
{
	q0=qq0, q1=qqv.i, q2=qqv.j, q3=qqv.k;
}

CQuat::CQuat(double qq0, double qq1, double qq2, double qq3)
{
	q0=qq0, q1=qq1, q2=qq2, q3=qq3;
}

CQuat::CQuat(const double *pdata)
{
	q0=*pdata++, q1=*pdata++, q2=*pdata++, q3=*pdata++;
}

CQuat CQuat::operator+(const CVect3 &phi) const
{
	CQuat qtmp = rv2q(-phi);
	return qtmp*(*this);
}

CQuat CQuat::operator-(const CVect3 &phi) const
{
	CQuat qtmp = rv2q(phi);
	return qtmp*(*this);
}

CVect3 CQuat::operator-(const CQuat &quat) const
{
	CQuat dq;
	
	dq = quat*(~(*this));
	if(dq.q0<0)
	{
		dq.q0=-dq.q0, dq.q1=-dq.q1, dq.q2=-dq.q2, dq.q3=-dq.q3;
	}
	double n2 = acos(dq.q0), f;
	if( sign(n2)!=0 )
	{
		f = 2.0/(sin(n2)/n2);
	}
	else
	{
		f = 2.0;
	}
	return CVect3(dq.q1,dq.q2,dq.q3)*f;
}

CQuat CQuat::operator*(const CQuat &quat) const
{
	CQuat qtmp;
	qtmp.q0 = q0*quat.q0 - q1*quat.q1 - q2*quat.q2 - q3*quat.q3;
	qtmp.q1 = q0*quat.q1 + q1*quat.q0 + q2*quat.q3 - q3*quat.q2;
	qtmp.q2 = q0*quat.q2 + q2*quat.q0 + q3*quat.q1 - q1*quat.q3;
	qtmp.q3 = q0*quat.q3 + q3*quat.q0 + q1*quat.q2 - q2*quat.q1;
	return qtmp;
}

CQuat& CQuat::operator*=(const CQuat &quat)
{
	return (*this=*this*quat);
}

CQuat& CQuat::operator-=(const CVect3 &phi)
{
	CQuat qtmp = rv2q(phi);
	return (*this=qtmp*(*this));
}

CQuat operator~(const CQuat &q)
{
	return CQuat(q.q0,-q.q1,-q.q2,-q.q3);
}

CVect3 CQuat::operator*(const CVect3 &v) const
{
	CQuat qtmp;
	CVect3 vtmp;
	qtmp.q0 =         - q1*v.i - q2*v.j - q3*v.k;
	qtmp.q1 = q0*v.i           + q2*v.k - q3*v.j;
	qtmp.q2 = q0*v.j           + q3*v.i - q1*v.k;
	qtmp.q3 = q0*v.k           + q1*v.j - q2*v.i;
	vtmp.i = -qtmp.q0*q1 + qtmp.q1*q0 - qtmp.q2*q3 + qtmp.q3*q2;
	vtmp.j = -qtmp.q0*q2 + qtmp.q2*q0 - qtmp.q3*q1 + qtmp.q1*q3;
	vtmp.k = -qtmp.q0*q3 + qtmp.q3*q0 - qtmp.q1*q2 + qtmp.q2*q1;
	return vtmp;
}

void CQuat::SetYaw(double yaw)
{
	CVect3 att = q2att(*this);
	att.k = yaw;
	*this = a2qua(att);
}

double normlize(CQuat *q)
{
	double nq=sqrt(q->q0*q->q0+q->q1*q->q1+q->q2*q->q2+q->q3*q->q3);
	q->q0 /= nq, q->q1 /= nq, q->q2 /= nq, q->q3 /= nq;
	return nq;
}

CVect3 q2rv(const CQuat &q)
{
	CQuat dq;
	dq = q;
	if(dq.q0<0)  { dq.q0=-dq.q0, dq.q1=-dq.q1, dq.q2=-dq.q2, dq.q3=-dq.q3; }
	if(dq.q0>1.0) dq.q0=1.0;
	double n2 = acos(dq.q0), f;
	if(n2>1.0e-20)
	{
		f = 2.0/(sin(n2)/n2);
	}
	else
	{
		f = 2.0;
	}
	return CVect3(dq.q1,dq.q2,dq.q3)*f;
}

CVect3 qq2phi(const CQuat &qcalcu, const CQuat &qreal)
{
    return q2rv(qreal*(~qcalcu));
}

CQuat addmu(const CQuat &q, const CVect3 &mu)
{
	return q*rv2q(mu);
}

CQuat UpDown(const CQuat &q)
{
	CVect3 att = q2att(q);
	att.i = -att.i; att.j += PI;
	return a2qua(att);
}

//***************************  class CMat3  *********************************/
CMat3::CMat3(void)
{
}

CMat3::CMat3(double xyz)
{
	e00=e01=e02 =e10=e11=e12 =e20=e21=e22 =xyz;
}

CMat3::CMat3(const double *pxyz)
{
	e00=*pxyz++,e01=*pxyz++,e02=*pxyz++,
	e10=*pxyz++,e11=*pxyz++,e12=*pxyz++,
	e20=*pxyz++,e21=*pxyz++,e22=*pxyz  ;
}

CMat3::CMat3(const float *pxyz)
{
	e00=*pxyz++,e01=*pxyz++,e02=*pxyz++,
	e10=*pxyz++,e11=*pxyz++,e12=*pxyz++,
	e20=*pxyz++,e21=*pxyz++,e22=*pxyz  ;
}

CMat3::CMat3(double xx, double yy, double zz)
{
	e00=xx, e11=yy, e22=zz;
	e01=e02 =e10=e12 =e20=e21 =0.0;
}

CMat3::CMat3(double xx, double xy, double xz, 
		  double yx, double yy, double yz,
		  double zx, double zy, double zz )
{
	e00=xx,e01=xy,e02=xz; e10=yx,e11=yy,e12=yz; e20=zx,e21=zy,e22=zz;
}

CMat3::CMat3(const CVect3 &v0, const CVect3 &v1, const CVect3 &v2, BOOL isrow)
{
	if(isrow) {
		e00 = v0.i, e01 = v0.j, e02 = v0.k;
		e10 = v1.i, e11 = v1.j, e12 = v1.k;
		e20 = v2.i, e21 = v2.j, e22 = v2.k;
	}
	else {
		e00 = v0.i, e01 = v1.i, e02 = v2.i;
		e10 = v0.j, e11 = v1.j, e12 = v2.j;
		e20 = v0.k, e21 = v1.k, e22 = v2.k;
	}
}

CVect3 sv2att(const CVect3 &fb, double yaw0, const CVect3 &fn)
{
    CVect3 phi = fb*fn;
    double afa = acos(dot(fn,fb)/norm(fn)/norm(fb)), nphi = norm(phi);
	CVect3 att = q2att(rv2q(phi*(nphi<1.0e-10?1.0:afa/nphi)));   att.k = yaw0;
	return att;  // return C^n_b
}

CVect3 dv2att(const CVect3 &va1, const CVect3 &va2, const CVect3 &vb1, const CVect3 &vb2)
{
	CVect3 a=va1*va2, b=vb1*vb2, aa=a*va1, bb=b*vb1;
	if(IsZero(va1)||IsZero(a)||IsZero(aa)||IsZero(vb1)||IsZero(b)||IsZero(bb)) return O31;
	CMat3 Ma(va1/norm(va1),a/norm(a),aa/norm(aa)), Mb(vb1/norm(vb1),b/norm(b),bb/norm(bb));
	return m2att((~Ma)*(Mb));  // return C^a_b -> att
}

CVect3 mv2att(int n, const CVect3 *vai, const CVect3 *vbi, ...)
{
	psinsassert(n>=2);
	CMat3 A(0.0);
	va_list vl;
	va_start(vl, vai);  vbi = va_arg(vl, CVect3*);  
	for(int i=0; i<n; i++)
	{ A += vxv(*vai,*vbi);  vai = va_arg(vl, CVect3*);  vbi = va_arg(vl, CVect3*); }
	va_end(vl);
	return m2att(sfoam(A));  // return C^a_b -> att
}

CVect3 vn2att(const CVect3 &vn)
{
	double vel = normXY(vn);
	if(vel<1.0e-6) return O31;
    return CVect3(atan2(vn.k, vel), 0, atan2(-vn.i, vn.j));
}

CVect3 atss(CVect3 &att, CVect3 &vn)
{
	CVect3 att1 = vn2att(vn);
	CVect3 as(att1.i-att.i, 0.0, att1.k-att.k);
	if(as.k>PI) as.k-=_2PI; else if(as.k<-PI) as.k+=_2PI;
	return as;
}

CMat3 operator-(const CMat3 &m)
{
	return CMat3(-m.e00,-m.e01,-m.e02,-m.e10,-m.e11,-m.e12,-m.e20,-m.e21,-m.e22);
}

CMat3 operator~(const CMat3 &m)
{
	return CMat3(m.e00,m.e10,m.e20, m.e01,m.e11,m.e21, m.e02,m.e12,m.e22);
}

CMat3 CMat3::operator*(const CMat3 &mat) const
{
	CMat3 mtmp;
	mtmp.e00 = e00*mat.e00 + e01*mat.e10 + e02*mat.e20;
	mtmp.e01 = e00*mat.e01 + e01*mat.e11 + e02*mat.e21;
	mtmp.e02 = e00*mat.e02 + e01*mat.e12 + e02*mat.e22;
	mtmp.e10 = e10*mat.e00 + e11*mat.e10 + e12*mat.e20;
	mtmp.e11 = e10*mat.e01 + e11*mat.e11 + e12*mat.e21;
	mtmp.e12 = e10*mat.e02 + e11*mat.e12 + e12*mat.e22;
	mtmp.e20 = e20*mat.e00 + e21*mat.e10 + e22*mat.e20;
	mtmp.e21 = e20*mat.e01 + e21*mat.e11 + e22*mat.e21;
	mtmp.e22 = e20*mat.e02 + e21*mat.e12 + e22*mat.e22;
	return mtmp;
}

CMat3 CMat3::operator+(const CMat3 &mat) const
{
	CMat3 mtmp;
	mtmp.e00 = e00 + mat.e00;  mtmp.e01 = e01 + mat.e01;  mtmp.e02 = e02 + mat.e02;  
	mtmp.e10 = e10 + mat.e10;  mtmp.e11 = e11 + mat.e11;  mtmp.e12 = e12 + mat.e12;  
	mtmp.e20 = e20 + mat.e20;  mtmp.e21 = e21 + mat.e21;  mtmp.e22 = e22 + mat.e22;  
	return mtmp;
}

CMat3& CMat3::operator+=(const CMat3 &mat)
{
	this->e00 += mat.e00;  this->e01 += mat.e01;  this->e02 += mat.e02;  
	this->e10 += mat.e10;  this->e11 += mat.e11;  this->e12 += mat.e12;  
	this->e20 += mat.e20;  this->e21 += mat.e21;  this->e22 += mat.e22;  
	return *this;
}

CMat3 CMat3::operator+(const CVect3 &v) const
{
	CMat3 mtmp=*this;
	mtmp.e00 += v.i;  mtmp.e11 += v.j;  mtmp.e22 += v.k;
	return mtmp;
}

CMat3& CMat3::operator+=(const CVect3 &v)
{
	this->e00 += v.i;  this->e11 += v.j;  this->e22 += v.k;  
	return *this;
}

CMat3 CMat3::operator-(const CMat3 &mat) const
{
	CMat3 mtmp;
	mtmp.e00 = e00 - mat.e00;  mtmp.e01 = e01 - mat.e01;  mtmp.e02 = e02 - mat.e02;  
	mtmp.e10 = e10 - mat.e10;  mtmp.e11 = e11 - mat.e11;  mtmp.e12 = e12 - mat.e12;  
	mtmp.e20 = e20 - mat.e20;  mtmp.e21 = e21 - mat.e21;  mtmp.e22 = e22 - mat.e22;  
	return mtmp;
}

CMat3 CMat3::operator*(double f) const
{
	return CMat3(e00*f,e01*f,e02*f, e10*f,e11*f,e12*f, e20*f,e21*f,e22*f);
}

double& CMat3::operator()(int r, int c)
{
	return (&this->e00)[r*3+c];
}

void CMat3::SetRow(int i, const CVect3 &v)
{
	double *p=&e00+i*3;
	*p=v.i, *(p+1)=v.j, *(p+2) = v.k;
}

void CMat3::SetClm(int i, const CVect3 &v)
{
	double *p=&e00+i;
	*p=v.i, *(p+3)=v.j, *(p+6) = v.k;
}

CVect3 CMat3::GetRow(int i) const
{
	const double *p=&e00+i*3;
	return CVect3(*p,*(p+1),*(p+2));
}

CVect3 CMat3::GetClm(int i) const
{
	const double *p=&e00+i;
	return CVect3(*p,*(p+3),*(p+6));
}

CMat3 Rot(double angle, char axis)
{
	double s=sin(angle), c=cos(angle);
	if(axis=='x'||axis=='X')		return CMat3(1,  0, 0,   0, c, -s,   0, s, c);
	else if(axis=='y'||axis=='Y')	return CMat3(c,  0, s,   0, 1,  0,  -s, 0, c);
	else							return CMat3(c, -s, 0,   s, c,  0,   0, 0, 1);
}

CVect3 rotz(const CVect3 &v, double angle)
{
	double s=sin(angle), c=cos(angle);
	return CVect3(v.i*c-v.j*s, v.i*s+v.j*c, v.k);
}

CMat3 rcijk(const CMat3 &m, int ijk)
{
	switch(ijk)
	{
//	case 012: return m; break;
	case 021: return CMat3(m.e00,m.e02,m.e01, m.e20,m.e22,m.e21, m.e10,m.e12,m.e11); break;
	case 102: return CMat3(m.e11,m.e10,m.e12, m.e01,m.e00,m.e02, m.e21,m.e20,m.e22); break;
	case 120: return CMat3(m.e11,m.e12,m.e10, m.e21,m.e22,m.e20, m.e01,m.e02,m.e00); break;
	case 201: return CMat3(m.e22,m.e20,m.e21, m.e02,m.e00,m.e01, m.e12,m.e10,m.e11); break;
	case 210: return CMat3(m.e22,m.e21,m.e20, m.e12,m.e11,m.e10, m.e02,m.e01,m.e00); break;
	}
	return m;
}

void symmetry(CMat3 &m)
{
	m.e01 = m.e10 = (m.e01+m.e10)*0.5;
	m.e02 = m.e20 = (m.e02+m.e20)*0.5;
	m.e12 = m.e21 = (m.e12+m.e21)*0.5;
}

CMat3 operator*(double f, const CMat3 &m)
{
	return CMat3(m.e00*f,m.e01*f,m.e02*f, m.e10*f,m.e11*f,m.e12*f, m.e20*f,m.e21*f,m.e22*f);
}

CVect3 CMat3::operator*(const CVect3 &v) const
{
	return CVect3(e00*v.i+e01*v.j+e02*v.k,e10*v.i+e11*v.j+e12*v.k,e20*v.i+e21*v.j+e22*v.k);
	stacksize();
}

CMat3 dotmul(const CMat3 &m1, const CMat3 &m2)
{
	CMat3 m;
	m.e00 = m1.e00*m2.e00, m.e01 = m1.e01*m2.e01, m.e02 = m1.e02*m2.e02; 
	m.e10 = m1.e10*m2.e10, m.e11 = m1.e11*m2.e11, m.e12 = m1.e12*m2.e12; 
	m.e20 = m1.e20*m2.e20, m.e21 = m1.e21*m2.e21, m.e22 = m1.e22*m2.e22; 
	return m;
}

double det(const CMat3 &m)
{
	return m.e00*(m.e11*m.e22-m.e12*m.e21) - m.e01*(m.e10*m.e22-m.e12*m.e20) + m.e02*(m.e10*m.e21-m.e11*m.e20);
}

double trace(const CMat3 &m)
{
	return (m.e00+m.e11+m.e22);
}

CMat3 pow(const CMat3 &m, int k)
{
	CMat3 mm = m;
	for(int i=1; i<k; i++)	mm = mm*m;
	return mm;
}

CQuat a2qua(double pitch, double roll, double yaw)
{
	pitch /= 2.0, roll /= 2.0, yaw /= 2.0;
    double	sp = sin(pitch), sr = sin(roll), sy = sin(yaw), 
			cp = cos(pitch), cr = cos(roll), cy = cos(yaw);
	CQuat qnb;
    qnb.q0 = cp*cr*cy - sp*sr*sy;
    qnb.q1 = sp*cr*cy - cp*sr*sy;
    qnb.q2 = cp*sr*cy + sp*cr*sy;
    qnb.q3 = cp*cr*sy + sp*sr*cy;
	return qnb;
}

CQuat a2qua(const CVect3 &att)
{
	return a2qua(att.i, att.j, att.k);
}

CMat3 a2mat(const CVect3 &att)
{
	double	si = sin(att.i), ci = cos(att.i),
			sj = sin(att.j), cj = cos(att.j),
			sk = sin(att.k), ck = cos(att.k);
	CMat3 Cnb;
	Cnb.e00 =  cj*ck - si*sj*sk;	Cnb.e01 =  -ci*sk;	Cnb.e02 = sj*ck + si*cj*sk;
	Cnb.e10 =  cj*sk + si*sj*ck;	Cnb.e11 =  ci*ck;	Cnb.e12 = sj*sk - si*cj*ck;
	Cnb.e20 = -ci*sj;				Cnb.e21 =  si;		Cnb.e22 = ci*cj;
	return Cnb;
}

CMat3 ar2mat(const CVect3 &attr)  // reversed Euler angles to DCM
{
	double	si = sin(attr.i), ci = cos(attr.i),
			sj = sin(attr.j), cj = cos(attr.j),
			sk = sin(attr.k), ck = cos(attr.k);
	CMat3 Cnb;
	Cnb.e00 =  cj*ck;	Cnb.e01 =  si*sj*ck-ci*sk;	Cnb.e02 = ci*sj*ck + si*sk;
	Cnb.e10 =  cj*sk;	Cnb.e11 =  si*sj*sk+ci*ck;	Cnb.e12 = ci*sj*sk - si*ck;
	Cnb.e20 = -sj;		Cnb.e21 =  si*cj;			Cnb.e22 = ci*cj;
	return Cnb;
}

CQuat ar2qua(const CVect3 &attr)
{
	return m2qua(ar2mat(attr));
}

CVect3 m2att(const CMat3 &Cnb)
{
	CVect3 att;
	att.i = asinEx(Cnb.e21);
	att.j = atan2Ex(-Cnb.e20, Cnb.e22);
	att.k = atan2Ex(-Cnb.e01, Cnb.e11);
	if(Cnb.e21>0.999999)       { att.j=0.0; att.k= atan2Ex(Cnb.e02,Cnb.e00); }
	else if(Cnb.e21<-0.999999) { att.j=0.0; att.k=-atan2Ex(Cnb.e02,Cnb.e00); }
	return att;
}

CVect3 m2attr(const CMat3 &Cnb)
{
	CVect3 attr;
	attr.i = atan2Ex(Cnb.e21, Cnb.e22);
	attr.j = asinEx(-Cnb.e20);
	attr.k = atan2Ex(Cnb.e10, Cnb.e00);
	return attr;
}

CVect3 q2attr(const CQuat &qnb)
{
	return m2attr(q2mat(qnb));
}

CVect3 m2rv(const CMat3 &Cnb)
{
	double phi=acos((trace(Cnb)-1.0)/2), afa;
	afa = (-1.0e-10<phi&&phi<1.0e-10) ? 1.0/2 : phi/(2*sin(phi));
	return CVect3(Cnb.e21-Cnb.e12, Cnb.e02-Cnb.e20, Cnb.e10-Cnb.e01)*afa;
}

CQuat m2qua(const CMat3 &Cnb)
{
	double q0, q1, q2, q3, qq4;
    if(Cnb.e00>=Cnb.e11+Cnb.e22)
	{
        q1 = 0.5*sqrt(1+Cnb.e00-Cnb.e11-Cnb.e22);  qq4 = 4*q1;
        q0 = (Cnb.e21-Cnb.e12)/qq4; q2 = (Cnb.e01+Cnb.e10)/qq4; q3 = (Cnb.e02+Cnb.e20)/qq4;
	}
    else if(Cnb.e11>=Cnb.e00+Cnb.e22)
	{
        q2 = 0.5*sqrt(1-Cnb.e00+Cnb.e11-Cnb.e22);  qq4 = 4*q2;
        q0 = (Cnb.e02-Cnb.e20)/qq4; q1 = (Cnb.e01+Cnb.e10)/qq4; q3 = (Cnb.e12+Cnb.e21)/qq4;
	}
    else if(Cnb.e22>=Cnb.e00+Cnb.e11)
	{
        q3 = 0.5*sqrt(1-Cnb.e00-Cnb.e11+Cnb.e22);  qq4 = 4*q3;
        q0 = (Cnb.e10-Cnb.e01)/qq4; q1 = (Cnb.e02+Cnb.e20)/qq4; q2 = (Cnb.e12+Cnb.e21)/qq4;
	}
    else
	{
        q0 = 0.5*sqrt(1+Cnb.e00+Cnb.e11+Cnb.e22);  qq4 = 4*q0;
        q1 = (Cnb.e21-Cnb.e12)/qq4; q2 = (Cnb.e02-Cnb.e20)/qq4; q3 = (Cnb.e10-Cnb.e01)/qq4;
	}
	double nq = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
	q0 /= nq; q1 /= nq; q2 /= nq; q3 /= nq;
	return CQuat(q0, q1, q2, q3);
}

CVect3 q2att(const CQuat &qnb)
{
	double	q11 = qnb.q0*qnb.q0, q12 = qnb.q0*qnb.q1, q13 = qnb.q0*qnb.q2, q14 = qnb.q0*qnb.q3, 
			q22 = qnb.q1*qnb.q1, q23 = qnb.q1*qnb.q2, q24 = qnb.q1*qnb.q3,     
			q33 = qnb.q2*qnb.q2, q34 = qnb.q2*qnb.q3,  
			q44 = qnb.q3*qnb.q3;
	CVect3 att;
	att.i = asinEx(2*(q34+q12));
	att.j = atan2Ex(-2*(q24-q13), q11-q22-q33+q44);
	att.k = atan2Ex(-2*(q23-q14), q11-q22+q33-q44);
	return att;
}

CMat3 q2mat(const CQuat &qnb)
{
	double	q11 = qnb.q0*qnb.q0, q12 = qnb.q0*qnb.q1, q13 = qnb.q0*qnb.q2, q14 = qnb.q0*qnb.q3, 
			q22 = qnb.q1*qnb.q1, q23 = qnb.q1*qnb.q2, q24 = qnb.q1*qnb.q3,     
			q33 = qnb.q2*qnb.q2, q34 = qnb.q2*qnb.q3,  
			q44 = qnb.q3*qnb.q3;
	CMat3 Cnb;
    Cnb.e00 = q11+q22-q33-q44,  Cnb.e01 = 2*(q23-q14),     Cnb.e02 = 2*(q24+q13),
	Cnb.e10 = 2*(q23+q14),      Cnb.e11 = q11-q22+q33-q44, Cnb.e12 = 2*(q34-q12),
	Cnb.e20 = 2*(q24-q13),      Cnb.e21 = 2*(q34+q12),     Cnb.e22 = q11-q22-q33+q44;
	return Cnb;
}

CMat3 Ka2Cba(const CMat3 &Ka, CVect3 &Sfa)
{
    CMat3 iKa = inv(Ka);
    Sfa = CVect3(norm(*(CVect3*)&iKa.e00), norm(*(CVect3*)&iKa.e10), norm(*(CVect3*)&iKa.e20));
    CMat3 Cba = ~(inv(diag(Sfa))*iKa);
	return Cba;
}

CMat3 Cba2Ka(const CMat3 &Cba, const CVect3 &Sfa)
{
   CMat3 Ka = inv(diag(Sfa)*(~Cba));
   return Ka;
}

void Ka22Kpn(const CVect3 &Ka1, const CVect3 &Ka2, CVect3 &Kap, CVect3 &Kan)
{
	CVect3 dKa1=G0*Ka2;
	Kap = dotmul(Ka1,One31+dKa1), Kan = dotmul(Ka1,One31-dKa1);
}

void Kpn2Ka2(const CVect3 &Kap, const CVect3 &Kan, CVect3 &Ka1, CVect3 &Ka2)
{
	Ka1 = (Kap+Kan)/2.0;
	Ka2 = (dotdiv(Kap,Ka1)-One31)*(1/G0);
}

void KgMdf(CMat3 &Kg, const double *dKg, int typ)
{
//	CMat3 IdKG = I33-(~(*(CMat3*)&FBXk.dd[12]));
//	sins.imu.Kg = IdKG*sins.imu.Kg;			// dKG
	CMat3 Cbg0, Cbg; CVect3 Sfg0, Sfg;
	if(typ) Cbg0 = Ka2Cba(Kg, Sfg0);
	double de00=1.0-dKg[0], de11=1.0-dKg[4], de22=1.0-dKg[8], e00, e01, e02, e10, e11, e12;
	   e00 =  de00  *Kg.e00 - dKg[3]*Kg.e10 - dKg[6]*Kg.e20;
	   e01 =  de00  *Kg.e01 - dKg[3]*Kg.e11 - dKg[6]*Kg.e21;
	   e02 =  de00  *Kg.e02 - dKg[3]*Kg.e12 - dKg[6]*Kg.e22;
	   e10 = -dKg[1]*Kg.e00 + de11  *Kg.e10 - dKg[7]*Kg.e20;
	   e11 = -dKg[1]*Kg.e01 + de11  *Kg.e11 - dKg[7]*Kg.e21;
	   e12 = -dKg[1]*Kg.e02 + de11  *Kg.e12 - dKg[7]*Kg.e22;
	Kg.e20 = -dKg[2]*Kg.e00 - dKg[5]*Kg.e10 + de22  *Kg.e20;
	Kg.e21 = -dKg[2]*Kg.e01 - dKg[5]*Kg.e11 + de22  *Kg.e21;
	Kg.e22 = -dKg[2]*Kg.e02 - dKg[5]*Kg.e12 + de22  *Kg.e22;
	Kg.e00=e00, Kg.e01=e01, Kg.e02=e02, Kg.e10=e10, Kg.e11=e11, Kg.e12=e12;
	if(typ) {  // bad
		Cbg = Ka2Cba(Kg, Sfg);
		Kg = typ==1 ? Cba2Ka(Cbg, Sfg0) : Cba2Ka(Cbg0, Sfg);
	}
}

void KaMdf(CMat3 &Ka, const double *dKa, int typ)
{
//	CMat3 IdKA(1.0-FBXk.dd[21],            0.0,           0.0,
//			      -FBXk.dd[22],1.0-FBXk.dd[24],           0.0,
//				  -FBXk.dd[23],   -FBXk.dd[25],1.0-FBXk.dd[26]);
//	sins.imu.Ka = IdKA*sins.imu.Ka;			// dKA
	CMat3 Cba0, Cba; CVect3 Sfa0, Sfa;
	if(typ) Cba0 = Ka2Cba(Ka, Sfa0);
	double de00=1.0-dKa[0], de11=1.0-dKa[3], de22=1.0-dKa[5];
	Ka.e20 = -dKa[2]*Ka.e00 - dKa[4]*Ka.e10 + de22  *Ka.e20;
	Ka.e21 = -dKa[2]*Ka.e01 - dKa[4]*Ka.e11 + de22  *Ka.e21;
	Ka.e22 = -dKa[2]*Ka.e02 - dKa[4]*Ka.e12 + de22  *Ka.e22;
	Ka.e10 = -dKa[1]*Ka.e00 + de11  *Ka.e10;
	Ka.e11 = -dKa[1]*Ka.e01 + de11  *Ka.e11;
	Ka.e12 = -dKa[1]*Ka.e02 + de11  *Ka.e12;
	Ka.e00 =  de00  *Ka.e00;
	Ka.e01 =  de00  *Ka.e01;
	Ka.e02 =  de00  *Ka.e02;
	if(typ) {
		Cba = Ka2Cba(Ka, Sfa);
		Ka = typ==1 ? Cba2Ka(Cba, Sfa0) : Cba2Ka(Cba0, Sfa);
	}
}

CMat3 adj(const CMat3 &m)
{
	CMat3 mtmp;
	mtmp.e00 =  (m.e11*m.e22-m.e12*m.e21);
	mtmp.e10 = -(m.e10*m.e22-m.e12*m.e20);
	mtmp.e20 =  (m.e10*m.e21-m.e11*m.e20);
	mtmp.e01 = -(m.e01*m.e22-m.e02*m.e21);
	mtmp.e11 =  (m.e00*m.e22-m.e02*m.e20);
	mtmp.e21 = -(m.e00*m.e21-m.e01*m.e20);
	mtmp.e02 =  (m.e01*m.e12-m.e02*m.e11);
	mtmp.e12 = -(m.e00*m.e12-m.e02*m.e10);
	mtmp.e22 =  (m.e00*m.e11-m.e01*m.e10);
	return mtmp;
}

CMat3 inv(const CMat3 &m)
{
	CMat3 adjm = adj(m);
	double detm = m.e00*adjm.e00 + m.e01*adjm.e10 + m.e02*adjm.e20;
	return adjm*(1.0/detm);
}

CVect3 diag(const CMat3 &m)
{
	return CVect3(m.e00, m.e11, m.e22);
}

CMat3 diag(const CVect3 &v)
{
	return CMat3(v.i,0,0, 0,v.j,0, 0,0,v.k);
}

CMat3 diag(double ii, double jj, double kk)
{
	if(jj>INFp5) jj=ii;  if(kk>INFp5) kk=jj;
	return CMat3(ii,0,0, 0,jj,0, 0,0,kk);
}

CMat3 askew(const CMat3 &m, int I)
{
	CMat3 m1;
	m1.e01=(m.e01-m.e10)/2;  m1.e02=(m.e02-m.e20)/2;  m1.e12=(m.e12-m.e21)/2;
	m1.e10=-m1.e01; m1.e20=-m1.e02; m1.e21=-m1.e12;
	if(I==0)  m1.e00=m1.e11=m1.e22=0.0;
	else if(I==1)  m1.e00=m1.e11=m1.e22=1.0;
	else  { m1.e00=m.e00, m1.e11=m.e11, m1.e22=m.e22; }
	return m1;
}

CMat3 MMT(const CMat3 &m1, const CMat3 &m2)
{
	CMat3 mtmp; const CMat3 *pm2;
	pm2 = (&m2==&I33) ? &m1 : &m2;
	mtmp.e00 = m1.e00*pm2->e00 + m1.e01*pm2->e01 + m1.e02*pm2->e02;
	mtmp.e01 = m1.e00*pm2->e10 + m1.e01*pm2->e11 + m1.e02*pm2->e12;
	mtmp.e02 = m1.e00*pm2->e20 + m1.e01*pm2->e21 + m1.e02*pm2->e22;
	mtmp.e11 = m1.e10*pm2->e10 + m1.e11*pm2->e11 + m1.e12*pm2->e12;
	mtmp.e12 = m1.e10*pm2->e20 + m1.e11*pm2->e21 + m1.e12*pm2->e22;
	mtmp.e22 = m1.e20*pm2->e20 + m1.e21*pm2->e21 + m1.e22*pm2->e22;
	mtmp.e10 = mtmp.e01;
	mtmp.e20 = mtmp.e02;
	mtmp.e21 = mtmp.e12;
	return mtmp;
}

double trMMT(const CMat3 &m1, const CMat3 &m2)
{
	const CMat3 *pm2;
	pm2 = (&m2==&I33) ? &m1 : &m2;
	return	m1.e00*pm2->e00 + m1.e01*pm2->e01 + m1.e02*pm2->e02 +
			m1.e10*pm2->e10 + m1.e11*pm2->e11 + m1.e12*pm2->e12 +
			m1.e20*pm2->e20 + m1.e21*pm2->e21 + m1.e22*pm2->e22;
}

double norm(const CMat3 &m)
{
	return sqrt(trMMT(m));
}

static double maxrt4(double b, double c, double d) // max real root for x^4+bx^2+cx+d=0
{
	double D=-8*b, E=-8*c, F=16*b*b-64*d, E2=E*E,
		A=D*D-3*F, B=D*F-9*E2, C=F*F-3*D*E2;// Delta=B*B-4*A*C;
	double sA, y, theta3, sAct3, sAst3, y1, y2, y3, sy1, sy2, sy3;
	if(A<0.0) { sA = 0.0,      y = 0.0; }
	else      { sA = sqrt(A),  y = (1.5*B/A-D)/sA; }  // y=(3*B-2*A*D)/(2*A*sA);
	if(y>1.0) y=1.0; else if(y<-1.0) y=-1.0;
	theta3 = acos(y)/3;
	sAct3 = sA*cos(theta3); sAst3 = sA*sqrt3*sin(theta3);
	y1 = D-2*sAct3      ;  sy1 = y1<0.0 ? 0.0 : sqrt(y1);
	y2 = D+  sAct3+sAst3;  sy2 = y2<0.0 ? 0.0 : sqrt(y2);
	y3 = D+  sAct3-sAst3;  sy3 = y3<0.0 ? 0.0 : sqrt(y3);
	if(E<0.0) sy1=-sy1;
	return (sy1+sy2+sy3)/(4*sqrt3);
}

CMat3 sfoam(const CMat3 &B, int iter)
{
    CMat3 adjBp=adj(~B), BBp=MMT(B), C;
    double detB=det(B), adjBp2=trMMT(adjBp), B2=trace(BBp), cnd,
		lambda, lambda2, kappa, zeta, Psi, dPsi, dlambda;
	if(B2<EPS||adjBp2<EPS) return I33;  // rank(B)<2
	cnd = detB*detB/B2/adjBp2;  // 1/cond(B)^2
	lambda = cnd>1.0e-12 ? maxrt4(-2*B2,-8*detB,B2*B2-4*adjBp2) : sqrt(B2+2*sqrt(adjBp2));
    for(int k=1; k<iter; k++)
	{
		lambda2 = lambda*lambda;
        kappa = (lambda2-B2)/2.0;
        zeta = kappa*lambda - detB;
		if(zeta<EPS) return I33;  // singular value s2+s3=0
		Psi = (lambda2-B2); Psi = Psi*Psi - 8.0*lambda*detB - 4.0*adjBp2;
		dPsi = 8.0*zeta;
		dlambda = Psi / dPsi;
        lambda = lambda - dlambda;
		if(fabs(dlambda/lambda)<1.0e-15) break;
    }
    C = ((kappa+B2)*B+lambda*adjBp-BBp*B) * (1.0/zeta);
	double nC=trMMT(C);
    return (nC<3.0-1.0e-6||nC>3.0+1.0e-6) ? I33 : C;
}

//***************************  class CMat  *********************************/
CMat::CMat(void)
{
#ifdef PSINS_MAT_COUNT
	#pragma message("  PSINS_MAT_COUNT")
	if(iMax<++iCount) iMax = iCount;
#endif
}
	
CMat::CMat(int row0, int clm0)
{
	if(clm0==0) clm0=row0;	// clm0==0 for square matrix
#ifdef PSINS_MAT_COUNT
	if(iMax<++iCount) iMax = iCount;
#endif
	row=row0; clm=clm0; rc=row*clm;
}

CMat::CMat(int row0, int clm0, double f)
{
#ifdef PSINS_MAT_COUNT
	if(iMax<++iCount) iMax = iCount;
#endif
	row=row0; clm=clm0; rc=row*clm;
	for(double *pd=dd, *pEnd=&dd[rc]; pd<pEnd; pd++)  *pd = f;
}

CMat::CMat(int row0, int clm0, double f, double f1, ...)
{
#ifdef PSINS_MAT_COUNT
	if(iMax<++iCount) iMax = iCount;
#endif
	row=row0; clm=clm0; rc=row*clm;
	va_list vl;
	va_start(vl, f);
	for(int i=0; i<rc; i++)
	{ if(f>2*INF) break;  dd[i] = f;  f = va_arg(vl, double);	}
	va_end(vl);
}

CMat::CMat(int row0, int clm0, const double *pf)
{
#ifdef PSINS_MAT_COUNT
	if(iMax<++iCount) iMax = iCount;
#endif
	row=row0; clm=clm0; rc=row*clm;
	memcpy(dd, pf, rc*sizeof(double));
}

CMat::CMat(int clm0, const CVect *pv, ...)
{
//	this->CMat(pv->row, clm0);
	row=pv->row; clm=clm0; rc=row*clm;
	va_list vl;
	va_start(vl, pv);
	for(int i=0; i<clm; i++) {
		SetClm(i, *pv);
		pv = va_arg(vl, const CVect*);
	}
	va_end(vl);
}

#ifdef PSINS_MAT_COUNT
int CMat::iCount=0, CMat::iMax=0;
CMat::~CMat(void)
{
	iCount--;
}
#endif

void CMat::Reset(int row0, int clm0)
{
	row=row0; clm=clm0; rc=row*clm;
}

void CMat::Clear(double f)
{
	for(double *p=dd, *pEnd=&dd[rc]; p<pEnd; p++) *p=f;
}

void CMat::ClearRow(int i, double f)
{
	for(double *p=&dd[i*clm], *pEnd=p+clm; p<pEnd; p++) *p=f;
}

void CMat::ClearClm(int j, double f)
{
	for(double *p=&dd[j], *pEnd=p+rc; p<pEnd; p+=clm) *p=f;
}

void CMat::ClearRC(int i, int j, double f)
{
	ClearRow(i, f), ClearClm(j==-1?i:j, f);
}

CMat CMat::operator*(const CMat &m0) const
{
#ifdef PSINS_MAT_COUNT
	++iCount;
#endif
	psinsassert(this->clm==m0.row);
	CMat mtmp(this->row,m0.clm);
	int m=this->row, k=this->clm, n=m0.clm;
	double *p=mtmp.dd; const double *p1i=this->dd, *p2=m0.dd;
	for(int i=0; i<m; i++,p1i+=k)
	{
		for(int j=0; j<n; j++)
		{
			double f=0.0; const double *p1is=p1i, *p1isEnd=&p1i[k], *p2sj=&p2[j];
			for(; p1is<p1isEnd; p1is++,p2sj+=n)
				f += (*p1is) * (*p2sj);
			*p++ = f;
		}
	}
	stacksize();
	return mtmp;
}

CMat CMat::operator*(const CMat3 &m0) const
{
#ifdef PSINS_MAT_COUNT
	++iCount;
#endif
	psinsassert(this->clm==3);
	CMat mtmp(3,3);  mtmp.SetMat3(0,0,m0);
	return *this*mtmp;
}

CVect CMat::operator*(const CVect &v) const
{
	psinsassert(this->clm==v.row);
	CVect vtmp(this->row);
	double *p=vtmp.dd, *pEnd=&vtmp.dd[vtmp.row]; const double *p1ij=this->dd, *p2End=&v.dd[v.row];
	for(; p<pEnd; p++)
	{
		double f=0.0; const double *p2j=v.dd;
		for(; p2j<p2End; p1ij++,p2j++)	f += (*p1ij) * (*p2j);
		*p = f;
	}
	stacksize();
	return vtmp;
}

CMat CMat::operator+(const CMat &m0) const
{
#ifdef PSINS_MAT_COUNT
	++iCount;
#endif
	psinsassert(row==m0.row&&clm==m0.clm);
	CMat mtmp(row,clm);
	double *p=mtmp.dd, *pEnd=&mtmp.dd[rc]; const double *p1=this->dd, *p2=m0.dd;
	while(p<pEnd)
	{ *p++ = (*p1++) + (*p2++); } 
	return mtmp;
}

CMat& CMat::operator+=(const CVect &v)
{
	psinsassert(row==v.row||clm==v.clm);
	int row1 = row+1;
	double *p=dd, *pEnd=&dd[rc];
	for(const double *p1=v.dd; p<pEnd; p+=row1, p1++)	*p += *p1;
	return *this;
}

CMat CMat::operator-(const CMat &m0) const
{
#ifdef PSINS_MAT_COUNT
	++iCount;
#endif
	psinsassert(row==m0.row&&clm==m0.clm);
	CMat mtmp(row,clm);
	double *p=mtmp.dd, *pEnd=&mtmp.dd[rc]; const double *p1=this->dd, *p2=m0.dd;
	while(p<pEnd)
	{ *p++ = (*p1++) - (*p2++); } 
	return mtmp;
}

CMat CMat::operator*(double f) const
{
#ifdef PSINS_MAT_COUNT
	++iCount;
#endif
	CMat mtmp(row,clm);
	double *p=mtmp.dd, *pEnd=&mtmp.dd[rc]; const double *p1=this->dd;
	while(p<pEnd)
	{ *p++ = (*p1++) * f; } 
	return mtmp;
}

CMat& CMat::operator=(double f)
{
	for(double *p=dd, *pEnd=&dd[rc]; p<pEnd; p++)  { *p = f; }
	return *this;
}

CMat& CMat::operator+=(const CMat &m0)
{
	psinsassert(row==m0.row&&clm==m0.clm);
	double *p=dd, *pEnd=&dd[rc]; const double *p1=m0.dd;
	while(p<pEnd)
	{ *p++ += *p1++; } 
	return *this;
}

CMat& CMat::operator-=(const CMat &m0)
{
	psinsassert(row==m0.row&&clm==m0.clm);
	double *p=dd, *pEnd=&dd[rc]; const double *p1=m0.dd;
	while(p<pEnd)
	{ *p++ -= *p1++; } 
	stacksize();
	return *this;
}

CMat& CMat::operator*=(double f)
{
	double *p=dd, *pEnd=&dd[rc];
	while(p<pEnd)
	{ *p++ *= f; } 
	return *this;
}

CMat& CMat::operator++()
{
	int row1=row+1;
	for(double *p=dd, *pEnd=&dd[rc]; p<pEnd; p+=row1)	*p += 1.0;
	return *this;
}

CMat operator~(const CMat &m0)
{
#ifdef PSINS_MAT_COUNT
	++CMat::iCount;
#endif
	CMat mtmp(m0.clm,m0.row);
	const double *pm=m0.dd;
	for(int i=0; i<m0.row; i++)
	{ for(int j=i; j<m0.rc; j+=m0.row) mtmp.dd[j] = *pm++; }
	return mtmp;
}

void symmetry(CMat &m)
{
	psinsassert(m.row==m.clm);
	double *prow0=&m.dd[1], *prowEnd=&m.dd[m.clm], *pclm0=&m.dd[m.clm], *pEnd=&m.dd[m.rc];
	for(int clm1=m.clm+1; prow0<pEnd; prow0+=clm1,pclm0+=clm1,prowEnd+=m.clm)
	{
		for(double *prow=prow0,*pclm=pclm0; prow<prowEnd; prow++,pclm+=m.clm)
			*prow = *pclm = (*prow+*pclm)*0.5;
	}
}

double trace(const CMat &m)
{
	psinsassert(m.row==m.clm);
	int row1 = m.row+1;
	double s = 0.0;
	for(const double *p=m.dd, *pEnd=&m.dd[m.rc]; p<pEnd; p+=row1)  s+=*p;
	return s;
}

CMat dotmul(const CMat &m1, const CMat &m2)
{
	psinsassert(m1.row==m2.row && m1.clm==m2.clm);
	CMat res(m1.row,m1.clm);
	double *p=res.dd;
	for(const double *p1=m1.dd, *p2=m2.dd, *pEnd=&m1.dd[m1.rc]; p1<pEnd; p1++,p2++,p++)  { *p = (*p1)*(*p2); }
	return res;
}

double& CMat::operator()(int r, int c)
{
	if(c<0) c = r;
	return this->dd[r*this->clm+c];
}

void CMat::SetRow(int i, double f, ...)
{
	va_list vl;
	va_start(vl, f);
	for(double *p=&dd[i*clm], *pEnd=p+clm; p<pEnd; p++)
	{ *p = f;  f = va_arg(vl, double);	}
	va_end(vl);
	return;
}

void CMat::SetRow(int i, const CVect &v)
{
	psinsassert(clm==v.clm);
	const double *p=v.dd;
	for(double *p1=&dd[i*clm],*pEnd=p1+clm; p1<pEnd; p++,p1++) *p1 = *p;
	return;
}

void CMat::SetClm(int j, double f, ...)
{
	va_list vl;
	va_start(vl, f);
	for(double *p=&dd[j], *pEnd=&p[rc]; p<pEnd; p+=clm)
	{ *p = f;  f = va_arg(vl, double);	}
	va_end(vl);
	return;
}

void CMat::SetClm(int j, const CVect &v)
{
	psinsassert(row==v.row);
	const double *p=v.dd;
	for(double *p1=&dd[j],*pEnd=&dd[rc]; p1<pEnd; p++,p1+=clm) *p1 = *p;
	return;
}

void CMat::SetClmVect3(int i, int j, const CVect3 &v)
{
	double *p=&dd[i*clm+j];
	*p = v.i; p += clm;
	*p = v.j; p += clm;
	*p = v.k;
}

void CMat::SetClmVect3(int i, int j, const CVect3 &v, const CVect3 &v1)
{
	double *p=&dd[i*clm+j];
	*p = v.i; *(p+1) = v1.i; p += clm;
	*p = v.j; *(p+1) = v1.j; p += clm;
	*p = v.k; *(p+1) = v1.k;
}

void CMat::SetClmVect3(int i, int j, const CVect3 &v, const CVect3 &v1, const CVect3 &v2)
{
	double *p=&dd[i*clm+j];
	*p = v.i; *(p+1) = v1.i; *(p+2) = v2.i; p += clm;
	*p = v.j; *(p+1) = v1.j; *(p+2) = v2.j; p += clm;
	*p = v.k; *(p+1) = v1.k; *(p+2) = v2.k; 
//	SetMat3(i,j, CMat3(v, v1, v2));
}

void CMat::SetRowVect3(int i, int j, const CVect3 &v)
{
	*(CVect3*)&dd[i*clm+j] = v;
}

void CMat::SetRowVect3(int i, int j, const CVect3 &v, const CVect3 &v1)
{
	CVect3 *p=(CVect3*)&dd[i*clm+j];
	*p++ = v;  *p = v1;
}

void CMat::SetRowVect3(int i, int j, const CVect3 &v, const CVect3 &v1, const CVect3 &v2)
{
	CVect3 *p=(CVect3*)&dd[i*clm+j];
	*p++ = v;  *p++ = v1;  *p = v2;
}

CVect3 CMat::GetRowVect3(int i, int j) const
{
	return *(CVect3*)&dd[i*clm+j];
}

CVect3 CMat::GetClmVect3(int i, int j) const
{
	CVect3 v;
	const double *p=&dd[i*clm+j];
	v.i = *p; p += clm;
	v.j = *p; p += clm;
	v.k = *p;
	return v;
}

void CMat::SetDiagVect3(int i, int j, const CVect3 &v)
{
	double *p=&dd[i*clm+j];
	*p = v.i;  p += clm+1;
	*p = v.j;  p += clm+1;
	*p = v.k;
}

CVect3 CMat::GetDiagVect3(int i, int j) const
{
	if(j==-1) j=i;
	CVect3 v;
	const double *p=&dd[i*clm+j];
	v.i = *p;  p += clm+1;
	v.j = *p;  p += clm+1;
	v.k = *p;
	return v;
}

void CMat::SetAskew(int i, int j, const CVect3 &v)
{
	double *p=&dd[i*clm+j];
	p[0] = 0.0; p[1] =-v.k; p[2] = v.j;  p += clm;
	p[0] = v.k; p[1] = 0.0; p[2] =-v.i;  p += clm;
	p[0] =-v.j; p[1] = v.i; p[2] = 0.0;
}

void CMat::SetMat3(int i, int j, const CMat3 &m)
{
	double *p=&dd[i*clm+j];
	*(CVect3*)p = *(CVect3*)&m.e00;  p += clm;
	*(CVect3*)p = *(CVect3*)&m.e10;  p += clm;
	*(CVect3*)p = *(CVect3*)&m.e20;
}

void CMat::SetMat3(int i, int j, const CMat3 &m, const CMat3 &m1)
{
	double *p=&dd[i*clm+j];
	*(CVect3*)p = *(CVect3*)&m.e00;  *(CVect3*)(p+3) = *(CVect3*)&m1.e00;  p += clm;
	*(CVect3*)p = *(CVect3*)&m.e10;  *(CVect3*)(p+3) = *(CVect3*)&m1.e10;  p += clm;
	*(CVect3*)p = *(CVect3*)&m.e20;  *(CVect3*)(p+3) = *(CVect3*)&m1.e20;  
}

void CMat::SetMat3(int i, int j, const CMat3 &m, const CMat3 &m1, const CMat3 &m2)
{
	double *p=&dd[i*clm+j];
	*(CVect3*)p = *(CVect3*)&m.e00;  *(CVect3*)(p+3) = *(CVect3*)&m1.e00;  *(CVect3*)(p+6) = *(CVect3*)&m2.e00;  p += clm;
	*(CVect3*)p = *(CVect3*)&m.e10;  *(CVect3*)(p+3) = *(CVect3*)&m1.e10;  *(CVect3*)(p+6) = *(CVect3*)&m2.e10;  p += clm;
	*(CVect3*)p = *(CVect3*)&m.e20;  *(CVect3*)(p+3) = *(CVect3*)&m1.e20;  *(CVect3*)(p+6) = *(CVect3*)&m2.e20;  
}

CMat3 CMat::GetMat3(int i, int j) const
{
	if(j==-1) j=i;
	CMat3 m;
	const double *p=&dd[i*clm+j];
	*(CVect3*)&m.e00 = *(CVect3*)p;  p += clm;
	*(CVect3*)&m.e10 = *(CVect3*)p;  p += clm;
	*(CVect3*)&m.e20 = *(CVect3*)p;
	return m;
}

void CMat::SubAddMat3(int i, int j, const CMat3 &m)
{
	double *p=&dd[i*clm+j];
	*(CVect3*)p += *(CVect3*)&m.e00;  p += clm;
	*(CVect3*)p += *(CVect3*)&m.e10;  p += clm;
	*(CVect3*)p += *(CVect3*)&m.e20;
}

CVect CMat::GetRow(int i) const
{
	CVect v(1, clm);
	const double *p1=&dd[i*clm], *pEnd=p1+clm;
	for(double *p=v.dd; p1<pEnd; p++,p1++) *p = *p1;
	return v;
}

void CMat::GetRow(CVect &v, int i)
{
	v.row=1, v.clm=v.rc=clm;
	const double *p1=&dd[i*clm], *pEnd=p1+clm;
	for(double *p=v.dd; p1<pEnd; p++,p1++) *p = *p1;
}

CVect CMat::GetClm(int j) const
{
	CVect v(row, 1);
	const double *p1=&dd[j], *pEnd=&dd[rc];
	for(double *p=v.dd; p1<pEnd; p++,p1+=clm) *p = *p1;
	return v;
}

CMat CMat::GetClm(int j1, int j2, ...) const
{
	va_list vl;
	va_start(vl, j1); int j=j1, n;
	for(n=0; n<MMD; n++) { 
		if(j<0) break;  j=va_arg(vl, int);
	}
	va_end(vl);
	//
	CMat m(row, n);
	va_start(vl, j1); j=j1;
	for(int i=0; i<n; i++) {
		const double *p1=&dd[j], *pEnd=&dd[rc];
		for(double *p=&m.dd[i]; p1<pEnd; p+=n,p1+=clm) *p = *p1;
		j=va_arg(vl, int);
	}
	va_end(vl);
	return m;
}

void CMat::GetClm(CVect &v, int j)
{
	v.row=v.rc=row, v.clm=1;
	const double *p1=&dd[j], *pEnd=&dd[rc];
	for(double *p=v.dd; p1<pEnd; p++,p1+=clm) *p = *p1;
}

void CMat::ZeroRow(int i)
{
	for(double *p=&dd[i*clm],*pEnd=p+clm; p<pEnd; p++) *p = 0.0;
}

void CMat::ZeroClm(int j)
{
	for(double *p=&dd[j],*pEnd=&dd[rc]; p<pEnd; p+=clm) *p = 0.0;
}

void CMat::ZeroRC(int i, int j)
{
	ZeroRow(i), ZeroClm(j==-1?i:j);
}

void CMat::SetDiag(double f, ...)
{
	*this = CMat(this->row, this->clm, 0.0);
	va_list vl;
	va_start(vl, f);
	double *p=dd, *pEnd=&dd[rc];
	for(int row1=row+1; p<pEnd; p+=row1)
	{ if(f>2*INF) break;  *p = f;  f = va_arg(vl, double);	}
	va_end(vl);
}

void CMat::SetDiag2(double f, ...)
{
	*this = CMat(this->row, this->clm, 0.0);
	va_list vl;
	va_start(vl, f);
	double *p=dd, *pEnd=&dd[rc];
	for(int row1=row+1; p<pEnd; p+=row1)
	{ if(f>2*INF) break;  *p = f*f;  f = va_arg(vl, double);	}
	va_end(vl);
}

void CMat::SetAscend(double f0, double df)
{
	for(int i=0; i<rc; i++, f0+=df)  dd[i]=f0;
}

double norm1(const CMat &m)
{
	return norm1(&m.dd[0], m.rc);
}

double normInf(const CMat &m)
{
	return normInf(&m.dd[0], m.rc);
}

CVect diag(const CMat &m)
{
	int row1 = m.row+1;
	CVect vtmp(m.row,1);
	double *p=vtmp.dd, *pEnd=&vtmp.dd[vtmp.row];
	for(const double *p1=m.dd; p<pEnd; p++, p1+=row1)	*p = *p1;
	return vtmp;
}

CMat eye(int n)
{
	CMat m(n,n, 0.0);
	double *p=m.dd, *pEnd=&m.dd[m.rc];
	for(n=n+1; p<pEnd; p+=n)  *p=1.0;
	return m;
}

CMat MatExam(int i, int j)
{
	CMat m(i,j);
	for(i=0; i<m.rc; i++) m.dd[i]=i;
	return m;
}

CMat inv4(const CMat &m)
{
	psinsassert(m.clm==m.row&&m.clm==4);
	CMat3 A11(m.dd[0], m.dd[1], m.dd[2], m.dd[4], m.dd[5], m.dd[6], m.dd[8], m.dd[9], m.dd[10]), iA11;
	CVect3 A12(m.dd[3], m.dd[7], m.dd[11]), A21(m.dd[12], m.dd[13], m.dd[14]);
	double A22=m.dd[15], iA22;
	iA11 = inv(A11);  iA22 = 1.0/(A22-dot(A21*iA11,A12));
	CMat M(4,4);
	M.SetMat3(0,0, iA11+vxv(iA11*A12*iA22,A21)*iA11);  // by using matrix-inversion-lemma
	M.SetClmVect3(0,3, -iA11*A12*iA22);
	M.SetRowVect3(3,0, -iA22*A21*iA11);
	M.dd[15]=iA22;
	return M;
}

CMat inv6(const CMat &m)
{
	psinsassert(m.clm==m.row&&m.clm==6);
	CMat3 A11=m.GetMat3(0,0), iA11, A12=m.GetMat3(0,3), A21=m.GetMat3(3,0), A22=m.GetMat3(3,3), iA22;
	iA11 = inv(A11);  iA22 = inv(A22-A21*iA11*A12);
	CMat M(6,6);
	M.SetMat3(0,0, iA11+iA11*A12*iA22*A21*iA11);  // by using matrix-inversion-lemma
	M.SetMat3(0,3, -iA11*A12*iA22);
	M.SetMat3(3,0, -iA22*A21*iA11);
	M.SetMat3(3,3, iA22);
	return M;
}

CVect lss(const CMat &A, const CVect &y)
{
	CMat AT=~A;
	if(A.clm==4)
		return inv4(AT*A)*(AT*y);
	else if(A.clm==6)
		return inv6(AT*A)*(AT*y);
	else //error
		return Onen1;
}

void RowMul(CMat &m, const CMat &m0, const CMat &m1, int r, int fast)
{
	psinsassert(m0.clm==m1.row);
	int rc0=r*m0.clm; fast=(r>=fast);
	double *p=&m.dd[rc0], *pEnd=p+m0.clm; const double *p0=&m0.dd[rc0], *p0End=p0+m0.clm, *p1j=m1.dd;
	for(; p<pEnd; p++,p1j++)
	{
		if(fast) { *p=p1j[r*m1.row]; continue; }
		double f=0.0; const double *p0j=p0, *p1jk=p1j;
		for(; p0j<p0End; p0j++,p1jk+=m1.clm)	 f += (*p0j) * (*p1jk);
		*p = f;
	}
}

void RowMulT(CMat &m, const CMat &m0, const CMat &m1, int r, int fast)
{
	psinsassert(m0.clm==m1.clm);
	int rc0=r*m0.clm, ifast=0;
	double *p=&m.dd[rc0], *pEnd=p+m0.clm; const double *p0=&m0.dd[rc0], *p0End=p0+m0.clm, *p1jk=m1.dd;
	for(; p<pEnd; p++,ifast++)
	{
		if(ifast>=fast) { *p=p0[ifast]; p1jk+=m1.clm; continue; }
		double f=0.0; const double *p0j=p0;
		for(; p0j<p0End; p0j++,p1jk++)	 f += (*p0j) * (*p1jk);
		*p = f;
	}
}

CMat diag(const CVect &v)
{
#ifdef PSINS_MAT_COUNT
	++CMat::iCount;
#endif
	int rc = v.row>v.clm ? v.row : v.clm, rc1=rc+1;
	CMat mtmp(rc,rc,0.0);
	double *p=mtmp.dd;
	for(const double *p1=v.dd, *p1End=&v.dd[rc]; p1<p1End; p+=rc1, p1++)	*p = *p1;
	return mtmp;
}

void DVMDVafa(const CVect &V, CMat &M, double afa)
{
	psinsassert(V.rc==M.row&&M.row==M.clm);
	int i = 0;
	const double *pv = V.dd;
	for(double vi=*pv, viafa=vi*afa; i<M.clm; i++,pv++,vi=*pv,viafa=vi*afa)
	{
		for(double *prow=&M.dd[i*M.clm],*prowEnd=prow+M.clm,*pclm=&M.dd[i]; prow<prowEnd; prow++,pclm+=M.row)
		{
			*prow *= vi;
			*pclm *= viafa;
		}
	}
}

//***************************  class CVect  *********************************/
CVect::CVect(void)
{
}

CVect::CVect(int row0, int clm0)
{
	if(clm0==1) { row=row0; clm=1;   }
	else		{ row=1;    clm=clm0;}
	rc = row*clm;
}

CVect::CVect(int row0, double f)
{
	row=row0; clm=1; rc=row*clm;
	for(int i=0;i<row;i++) dd[i]=f;
}

CVect::CVect(int row0, const double *pf)
{
	row=row0; clm=1; rc=row*clm;
	memcpy(dd, pf, row*sizeof(double));
}

CVect::CVect(int row0, double f, double f1, ...)
{
	row=row0; clm=1; rc=row*clm;
	psinsassert(row<=MMD&&clm<=MMD);
	dd[0] = f;
	va_list vl;
	va_start(vl, f1);
	for(int i=1, rc=row>clm?row:clm; i<rc; i++)
	{ if(f>2*INF) break;  dd[i] = f1;  f1 = va_arg(vl, double);	}
	va_end(vl);
}

CVect::CVect(const CVect3 &v)
{
	row=3; clm=1; rc=row*clm;
	dd[0]=v.i; dd[1]=v.j; dd[2]=v.k;
}

CVect::CVect(const CVect3 &v1, const CVect3 v2)
{
	row=6; clm=1; rc=row*clm;
	dd[0]=v1.i; dd[1]=v1.j; dd[2]=v1.k;
	dd[3]=v2.i; dd[4]=v2.j; dd[5]=v2.k;
}

void CVect::Clear(void)
{
	for(int i=0;i<rc;i++) dd[i]=0.0;
}

void CVect::Reset(int row0, int clm0)
{
	if(clm0==1) { row=row0; clm=1;   }
	else		{ row=1;    clm=clm0;}
	rc = row*clm;
}

CVect operator-(const CVect &v)
{
	CVect vtmp=v;
	for(double *p=&vtmp.dd[0], *pEnd=&vtmp.dd[vtmp.rc]; p<pEnd; p++) *p=-*p;
	return vtmp;
}

CVect operator~(const CVect &v)
{
	CVect vtmp=v;
	vtmp.row=v.clm; vtmp.clm=v.row;
	return vtmp;
}

CVect CVect::operator*(const CMat &m) const
{
	psinsassert(clm==m.row);
	CVect vtmp(row,clm);
	double *p=vtmp.dd; const double *p1End=&dd[clm];
	for(int j=0; j<clm; p++,j++)
	{
		double f=0.0; const double *p1j=dd, *p2jk=&m.dd[j];
		for(; p1j<p1End; p1j++,p2jk+=m.clm)	 f += (*p1j) * (*p2jk);
		*p = f;
	}
	return vtmp;
}

CMat CVect::operator*(const CVect &v) const
{
#ifdef MAT_STATISTIC
	++CMat::iCount;
#endif
	psinsassert(clm==v.row);
	CMat mtmp(row,v.clm);
	if(row==1 && v.clm==1)  // (1x1) = (1xn)*(nx1)
	{
		double f = 0.0;
		for(int i=0; i<clm; i++)  f += dd[i]*v.dd[i];
		mtmp.dd[0] = f;
	}
	else    // (nxn) = (nx1)*(1xn)
	{
		double *p=mtmp.dd;
		for(const double *p1=&dd[0],*p1End=&dd[rc],*p2End=&v.dd[rc]; p1<p1End; p1++)
		{
			for(const double *p2=&v.dd[0]; p2<p2End; p2++)  *p++ = *p1 * *p2;
		}
	}
	stacksize();
	return mtmp;
}

CVect CVect::operator+(const CVect &v) const
{
	psinsassert(row==v.row&&clm==v.clm);
	const double *p2=v.dd, *p1=dd, *p1End=&dd[rc];
	CVect vtmp(row,clm);
	for(double *p=vtmp.dd; p1<p1End; p++,p1++,p2++)  { *p=*p1+*p2; }
	return vtmp;
}

CVect CVect::operator-(const CVect &v) const
{
	psinsassert(row==v.row&&clm==v.clm);
	const double *p2=v.dd, *p1=dd, *p1End=&dd[rc];
	CVect vtmp(row,clm);
	for(double *p=vtmp.dd; p1<p1End; p++,p1++,p2++)  { *p=*p1-*p2; }
	return vtmp;
}
	
CVect CVect::operator*(double f) const
{
	CVect vtmp(row,clm);
	const double *p1=dd,*p1End=&dd[rc];
	for(double *p=vtmp.dd; p1<p1End; p++,p1++)  { *p=*p1*f; }
	stacksize();
	return vtmp;
}

CVect& CVect::operator=(double f)
{
	for(double *p=dd, *pEnd=&dd[rc]; p<pEnd; p++)  { *p = f; }
	return *this;
}

CVect& CVect::operator=(const double *pf)
{
	for(double *p=dd, *pEnd=&dd[rc]; p<pEnd; p++,pf++)  { *p = *pf; }
	return *this;
}

CVect& CVect::operator=(const CMat3 &m)
{
	row=9; clm=1; rc=9;
	memcpy(dd, &m.e00, 9*sizeof(double));
	return *this;
}

CVect& CVect::operator+=(const CVect &v)
{
	psinsassert(row==v.row&&clm==v.clm);
	const double *p1 = v.dd;
	for(double *p=dd, *pEnd=&dd[rc]; p<pEnd; p++,p1++)  { *p += *p1; }
	return *this;
}

CVect& CVect::operator-=(const CVect &v)
{
	psinsassert(row==v.row&&clm==v.clm);
	const double *p1 = v.dd;
	for(double *p=dd, *pEnd=&dd[rc]; p<pEnd; p++,p1++)  { *p -= *p1; }
	return *this;
}

CVect& CVect::operator*=(double f)
{
	for(double *p=dd, *pEnd=&dd[rc]; p<pEnd; p++)  { *p *= f; }
	return *this;
}

double dot(const CVect &v1, const CVect &v2)
{
	psinsassert(v1.row==v2.row && v1.clm==v2.clm);
	double res=0.0;
	for(const double *p1=v1.dd, *p2=v2.dd, *pEnd=&v1.dd[v1.rc]; p1<pEnd; p1++,p2++)  { res += (*p1)*(*p2); }
	return res;
}

CVect dotmul(const CVect &v1, const CVect &v2)
{
	psinsassert(v1.row==v2.row && v1.clm==v2.clm);
	CVect res(v1.row,v1.clm);
	double *p=res.dd;
	for(const double *p1=v1.dd, *p2=v2.dd, *pEnd=&v1.dd[v1.rc]; p1<pEnd; p1++,p2++,p++)  { *p = (*p1)*(*p2); }
	return res;
}

CVect pow(const CVect &v, int k)
{
	CVect pp = v;
	double *p, *pEnd=&pp.dd[pp.rc];
	for(int i=1; i<k; i++)
	{
		p=pp.dd;
		for(const double *p1=v.dd; p<pEnd; p++,p1++)
			*p *= *p1;
	}
	return pp;
}

CVect abs(const CVect &v)
{
	CVect res(v.row,v.clm);
	const double *p=v.dd, *pEnd=&v.dd[v.rc];
	for(double *p1=res.dd; p<pEnd; p++,p1++)  { *p1 = *p>0 ? *p : -*p; }
	return res;
}

void neg(CVect &v, const int idx[], int n)
{
	if(n<=0) n=MMD;
	for(int k=0; k<n; k++) {
		if(idx[k]<0) break;
		else if(idx[k]>=MMD) continue;  // skip
		v.dd[idx[k]] = -v.dd[idx[k]];
	}
}

void enlarge(CVect &v, double f, const int idx[], int n)
{
	if(n<=0) n=MMD;
	for(int k=0; k<n; k++) {
		if(idx[k]<0) break;
		else if(idx[k]>=MMD) continue;  // skip
		v.dd[idx[k]] = v.dd[idx[k]]*f;
	}
}

double norm(const CVect &v)
{
	return norm(&v.dd[0], v.rc);
}

double norm1(const CVect &v)
{
	return norm1(&v.dd[0], v.rc);
}

double normInf(const CVect &v)
{
	return normInf(&v.dd[0], v.rc);
}

double& CVect::operator()(int r)
{
	return this->dd[r];
}

void CVect::Set(double f, ...)
{
	psinsassert(rc<=MMD);
	va_list vl;
	va_start(vl, f);
	for(int i=0; i<rc; i++)
	{ if(f>2*INF) break;  dd[i] = f;  f = va_arg(vl, double);	}
	va_end(vl);
}

void CVect::Set2(double f, ...)
{
	psinsassert(rc<=MMD);
	va_list vl;
	va_start(vl, f);
	for(int i=0; i<rc; i++)
	{ if(f>2*INF) break;  dd[i] = f*f;  f = va_arg(vl, double);	}
	va_end(vl);
}

void CVect::SetVect3(int i, const CVect3 &v)
{
	*(CVect3*)&dd[i] = v;
}

void CVect::Set2Vect3(int i, const CVect3 &v)
{
	dd[i++]=v.i*v.i; dd[i++]=v.j*v.j; dd[i]=v.k*v.k; 
}

void CVect::SetAscend(double f0, double df)
{
	for(int i=0; i<rc; i++, f0+=df)  dd[i]=f0;
}

void CVect::SetBit(unsigned int bit, double f)
{
	for(int i=0; i<rc; i++)		// assert(rc<32)
		if(bit&(0x01<<i)) dd[i]=f;
}

void CVect::SetBit(unsigned int bit, const CVect3 &v)
{
	const double *p=&v.i;
	for(int i=0; i<rc; i++)		// assert(rc<32)
		if(bit&(0x01<<i)) { dd[i]=*p++;  if(p>&v.k) p=&v.i; }
}

void CVect::Seti2j(int i, int j, double val)
{
	for(double *p=&dd[i], *pEnd=&dd[mmin(j,MMD-1)]; p<=pEnd; p++) *p = val;
}

CVect3 CVect::GetVect3(int i) const
{
	return *(CVect3*)&dd[i]; 
}

double mean(const CVect &v)
{
	double m=0.0;
	for(const double *p=v.dd, *pend=&v.dd[v.rc]; p<pend; p++) m += *p;
	return m/v.rc;
}

CVect sort(const CVect &v)
{
	CVect vtmp=v;
	double *pi=vtmp.dd, *pj, *pend=&vtmp.dd[vtmp.rc];
	for(; pi<pend; pi++)
		for(pj=pi+1; pj<pend; pj++)
			if(*pi<*pj) swapt(*pi,*pj,double);
	return vtmp;
}

//***************************  class CPolyfit  *****************************/
CPolyfit::CPolyfit(void)
{
}

void CPolyfit::Init(double ts0, int n0, double spii)
{
	psinsassert(n0<=4);
	ts = ts0;  tk = 0.0;  n = n0+1;  // polynomial highest order x^n0
	for(int i=0; i<5; i++) {
		Xk[i] = Kk[i] = 0.0;
		for(int j=0; j<5; j++)  Pk[i][j] = 0.0;
		Pk[i][i] = spii*spii;
	}
	U = &Pk[0][0];  // multiuse
	inHk = 1;  byUD = 0;  Hk[0] = 1.0;  Rk = 1.0;
}

void CPolyfit::SetP(double spii, ...)
{
	va_list vl;
	va_start(vl, spii);
	for(int i=0; i<n; i++)
	{  Pk[i][i] = spii*spii;  spii = va_arg(vl, double);  }
	va_end(vl);
}

void CPolyfit::SetUD(double spii, ...)
{
	va_list vl;
	va_start(vl, spii);
	for(int i=0; i<n; i++)	{
		for(int j=0; j<n; j++) { U[i*n+j]=0.0; }  U[i*n+i] = 1.0; 
		D[i] = spii*spii;  spii = va_arg(vl, double);
	}
	va_end(vl);
	byUD = 1;
}

void CPolyfit::SetHk(double hi, ...)
{
	va_list vl;
	va_start(vl, hi);
	for(int i=0; i<n; i++)
	{  Hk[i] = hi;  hi = va_arg(vl, double);  }
	va_end(vl);
}

void CPolyfit::UpdateP(double afa)		// RLS for Kk&Pk only
{
	if(byUD==1) {
		MeasUD(U, D, Hk, Rk*afa, Kk, n); return;
	}
	int i, j;
	double Pxz[5], Pz;
	for(i=0; i<n; i++)  { Pxz[i]=0.0;  for(j=0; j<n; j++) Pxz[i]+=Pk[i][j]*Hk[j]; }  // Pxz=Pk*Hk'
	Pz=Rk*afa; for(i=0; i<n; i++)  Pz+=Hk[i]*Pxz[i];  // Pz=Hk*Pxz+Rk
	Pz=1.0/Pz; for(i=0; i<n; i++)  { Kk[i]=Pxz[i]*Pz; }  // Kk=Pxz*Pz^-1
	for(i=0; i<n; i++) {  // Pk=Pk_1-Kk*Pxz';
		Pk[i][i]-=Kk[i]*Pxz[i];
		for(j=i+1; j<n; j++)  { Pk[i][j]-=Kk[i]*Pxz[j]; Pk[j][i]=Pk[i][j]; }
	}
}

void CPolyfit::Update(double Zk, double afa)
{
	int i;
	double r=Zk;
	tk += ts;
	if(inHk) { for(i=1; i<n; i++)  Hk[i]=Hk[i-1]*tk; }
	UpdateP(afa);
	for(i=0; i<n; i++) r -= Hk[i]*Xk[i];
	for(i=0; i<n; i++) Xk[i] += Kk[i]*r;
}

double CPolyfit::eval(double t)
{
	double val=0.0, ti=1.0;  // ti=t^i
	if(inHk) for(int i=0; i<n; i++) { val+=Xk[i]*ti; ti*=t; }
	else     for(int i=0; i<n; i++) { val+=Hk[i]*Xk[i]; }
	return val;
}

//***************************  class CPolyfit3  ****************************/
CPolyfit3::CPolyfit3(void)
{
}

void CPolyfit3::Init(double ts0, int n0, double spii)
{
	CPolyfit::Init(ts0, n0, spii);
	for(int i=0; i<5; i++) Xkv[i]=O31;
}

void CPolyfit3::Update(const CVect3 &Zk, double afa)
{
	int i;	CVect3 r=Zk;
	tk += ts;
	if(inHk) { for(i=1; i<n; i++)  Hk[i]=Hk[i-1]*tk; }
	UpdateP(afa);
	for(i=0; i<n; i++) r -= Hk[i]*Xkv[i];
	for(i=0; i<n; i++) Xkv[i] += Kk[i]*r;
}

CVect3 CPolyfit3::eval(double t)
{
	CVect3 val=O31; double ti=1.0;  // ti=t^i
	if(inHk) for(int i=0; i<n; i++) { val+=Xkv[i]*ti; ti*=t; }
	else     for(int i=0; i<n; i++) { val+=Hk[i]*Xkv[i]; }
	return val;
}

//***************************  class CIIR  *********************************/
CIIR::CIIR(void)
{
}

CIIR::CIIR(double *b0, double *a0, int n0)
{
	psinsassert(n0<IIRnMax);
	for(int i=0; i<n0; i++)  { b[i]=b0[i]/a0[0]; a[i]=a0[i]; x[i]=y[i]=0.0; }
	n = n0;
}

double CIIR::Update(double x0)
{
//	a(1)*y(n) = b(1)*x(n) + b(2)*x(n-1) + ... + b(nb+1)*x(n-nb)
//                        - a(2)*y(n-1) - ... - a(na+1)*y(n-na)
	double y0 = 0.0;
	for(int i=n-1; i>0; i--)
	{
		x[i] = x[i-1]; y[i] = y[i-1];
		y0 += b[i]*x[i] - a[i]*y[i];
	}
	x[0] = x0;
	y0 += b[0]*x0;
	y[0] = y0;
	return y0;
}

//***************************  class CV3IIR  *********************************/
CIIRV3::CIIRV3(void)
{
}

CIIRV3::CIIRV3(double *b0, double *a0, int n0, double *b1, double *a1, int n1, 
			   double *b2, double *a2, int n2)
{
	iir0 = CIIR(b0, a0, n0);
	if(n1==0)	iir1 = iir0;	// iir1 the same as iir0
	else		iir1 = CIIR(b1, a1, n1);
	if(n2==0)	iir2 = iir0;	// iir2 the same as iir0
	else		iir2 = CIIR(b2, a2, n2);
}

CVect3 CIIRV3::Update(const CVect3 &x)
{
	return y = CVect3(iir0.Update(x.i), iir1.Update(x.j), iir2.Update(x.k));
}

//***************************  class CRAvar  *********************************/
CRAvar::CRAvar()
{
}

CRAvar::CRAvar(int nR0, int maxCount0)
{
	psinsassert(nR0<RAMAX);
	this->nR0 = nR0;
	for(int i=0; i<RAMAX; i++)  { Rmaxcount[i]=maxCount0, tau[i]=INF; }
}

void CRAvar::set(double r0, double tau, double rmax, double rmin, int i)
{
	this->R0[i] = r0*r0;
	this->tau[i] = tau;
	this->r0[i] = 0.0;  Rmaxflag[i] = Rmaxcount[i];
	this->Rmax[i] = rmax==0.0 ? 100.0*this->R0[i] : rmax*rmax;
	this->Rmin[i] = rmin==0.0 ?  0.01*this->R0[i] : rmin*rmin;
}

void CRAvar::set(const CVect3 &r0, const CVect3 &tau, const CVect3 &rmax, const CVect3 &rmin)
{
	const double *pr0=&r0.i, *ptau=&tau.i, *prmax=&rmax.i, *prmin=&rmin.i;
	for(int i=0; i<3; i++,pr0++,ptau++,prmax++,prmin++)
		set(*pr0, *ptau, *prmax, *prmin, i);
}

void CRAvar::set(const CVect &r0, const CVect &tau, const CVect &rmax, const CVect &rmin)
{
	const double *pr0=r0.dd, *ptau=tau.dd, *prmax=rmax.dd, *prmin=rmin.dd;
	for(int i=0; i<nR0; i++,pr0++,ptau++,prmax++,prmin++)
		set(*pr0, *ptau, *prmax, *prmin, i);
}

void CRAvar::Update(double r, double ts, int i)
{
	if(tau[i]>INFp5) return;
	double tstau = ts>tau[i] ? 1.0 : ts/tau[i];
	double dr2=r-r0[i]; dr2=dr2*dr2; r0[i]=r;
	if(dr2>R0[i]) R0[i]=dr2; else R0[i]=(1.0-tstau)*R0[i]+tstau*dr2;
	if(R0[i]<Rmin[i]) R0[i]=Rmin[i];
	if(R0[i]>Rmax[i]) {R0[i]=Rmax[i];Rmaxflag[i]=Rmaxcount[i];} else {Rmaxflag[i]-=Rmaxflag[i]>0;}
}

void CRAvar::Update(const CVect3 &r, double ts)
{
	const double *pr=&r.i;
	for(int i=0; i<3; i++,pr++)
		Update(*pr, ts, i);
}

void CRAvar::Update(const CVect &r, double ts)
{
	const double *pr=r.dd;
	for(int i=0; i<nR0; i++,pr++)
		Update(*pr, ts, i);
}

double CRAvar::operator()(int k) const
{
	return Rmaxflag[k] ? INF : sqrt(R0[k]);
}

//***************************  class CVAR  ***********************************/
CVAR::CVAR(int imax0, double data0)
{
	imax = mmin(imax0, VARMAX);
	for(ipush=0; ipush<imax; ipush++)	array[ipush] = data0;
	ipush = 0;
	mean = data0; var = 0.0;
}

double CVAR::Update(double data, BOOL isvar)
{
	array[ipush] = data;
	if(++ipush==imax) ipush=0;
	double *p0, *p1;
	for(mean=0.0,p0=&array[0],p1=&array[imax]; p0<p1; p0++)  mean+=*p0;
	mean /= imax;
	if (isvar)
	{
		for (var = 0.0, p0 = &array[0], p1 = &array[imax]; p0 < p1; p0++)  { double vi = *p0 - mean; var += vi*vi; }
		var /= imax - 1;
	}
	return var;
}

//***************************  class CVARn  ***********************************/
CVARn::CVARn(void)
{
	pData = NULL;
}

CVARn::CVARn(int row0, int clm0)
{
	row = row0, clm = clm0;
	pData = new double*[clm];
	pData[0] = new double[row*clm+5*clm];
	for(int i = 1; i < clm; i++) {
		pData[i] = pData[i - 1] + row;
	}
	pd = pData[clm-1]+row;  Sx = pd + clm;  Sx2 = Sx + clm;  mx = Sx2 + clm;  stdx = mx + clm;
	stdsf = sqrt((row - 1) / (row - 2.0));
	Reset();
}

CVARn::~CVARn(void)
{
	Deletep(pData[0]);
	Deletep(pData);
}

void CVARn::Reset(void)
{
	idxpush = rowcnt = 0;
	memset(pData[0], 0, row*clm*sizeof(double));
	memset(Sx, 0, 5*clm*sizeof(double));
}

BOOL CVARn::Update(const double *pf)
{
	if (!pData[0])  return FALSE;
	if (++rowcnt > row) rowcnt = row;
	int idxnext = (idxpush >= row - 1) ? 0 : idxpush + 1;
	for(int i = 0; i < clm; i++)
	{
		double f=*pf++;  if(f>1e5) f=1e5; else if(f<-1e5) f=-1e5;
		pData[i][idxpush] = f;
		Sx[i] += f - pData[i][idxnext];
		Sx2[i] += f*f - pData[i][idxnext] * pData[i][idxnext];
		mx[i] = Sx[i] / rowcnt;
		stdx[i] = sqrt(Sx2[i] / rowcnt - mx[i] * mx[i]) * stdsf;   // Dx = E(x^2) - (Ex)^2
	}
	if (++idxpush == row) {
		idxpush = 0;
	}
	return idxpush == 0;
}

BOOL CVARn::Update(double f, ...)
{
	va_list vl;
	va_start(vl, f);
	for(int i = 0; i < clm; i++)
	{
		pd[i] = f;
		f = va_arg(vl, double);
	}
	va_end(vl);
	return Update(pd);
}

//***************************  class CContLarge  *********************************/
CContLarge::CContLarge(void)
{
}

CContLarge::CContLarge(double large0, double dt0, int cnt0)
{
	large=large0; dt=dt0; cnt=cnt0; cnti=0; t0=-INF;
}
    
BOOL CContLarge::Update(double val, double t1)
{
	if(t1<INFp5) {
		if(t1-t0>dt) cnti=0;  // restart
		t0=t1;
	}
	if(val>large||val<-large) {
		if(++cnti>cnt)	{ cnti=0; return TRUE; }
	}
	return FALSE;
};

//***************************  class CAbnomalCnt  *********************************/
CAbnomalCnt::CAbnomalCnt(void)
{
}

CAbnomalCnt::CAbnomalCnt(int cntMax0, double tlast0, double valMax0, double valMin0)
{
	cntMax = cntMax0;  tlast = tlast0;  valMax = valMax0;  valMin = valMin0<-INFp5 ? -valMax : valMin0;
	cnt = abnFlag = 0;  t0 = 0.0;
}

BOOL CAbnomalCnt::Update(double val, double t)
{
	if(abnFlag && (++cnt>cntMax||(t-t0)>tlast)) abnFlag = FALSE;
	if(val<valMin||val>valMax)	{
		abnFlag = TRUE;  cnt = 0;  t0 = t;
	}
	return abnFlag;
}

//***************************  class CWzhold  *********************************/
CWzhold::CWzhold(void)
{
}

void CWzhold::Init(double maxw0, double ts0, double T0, int cntNP0)
{
	ts = ts0;  T = T0;	maxw = maxw0;	cntNP = cntNP0;
	meanw = meanwpre = meanwprepre = tstop = 0.0;
	Reset();
}
	
void CWzhold::Reset(void)
{
	t = val = 0.0;
	cntNi = cntPi = big = 0;
}
	
int CWzhold::Update(double wz)   // wz in rad/s
{
	retres=0;
	if((tstop+=ts)<3.0) {
		retres=-6;
	}
	else {
		if(wz<-maxw) {
			if(++cntNi>cntNP) { meanw=meanwpre=meanwprepre=tstop=0.0; Reset(); retres=-3; }
			else retres=-1;
			big++;
		}
		else if(wz>maxw) {
			if(++cntPi>cntNP) { meanw=meanwpre=meanwprepre=tstop=0.0; Reset(); retres=-4; }
			else retres=-2;
			big++;
		}
		else {
			cntNi = cntPi = 0;
			big--;
		}
		if(big<0) {
			big=0;
		}
		else if(big>10) {
			meanw=meanwpre=meanwprepre=0.0; Reset(); retres=-5;
		}
		if(retres>=-2) {
			val += wz;
			t += ts;
			if(t>T) {
				meanwprepre = meanwpre;
				meanwpre = meanw;
				meanw = val*ts/T;  // meanw in rad/s
				Reset();
				if(meanwpre<-EPS||meanwpre>EPS) {
					if(meanwprepre<-EPS||meanwprepre>EPS) retres=3;
					else retres = 2;
				}
				else {
					retres = 1;
				}
			}
		}
	}
	return retres;
};

//***************************  class CMaxMin  *********************************/
CMaxMin::CMaxMin(int cnt00, int pre00, float f0)
{
	Init(cnt00, pre00, f0);
}

void CMaxMin::Init(int cnt00, int pre00, float f0)
{
	max0=f0, min0=-f0, maxpre0=f0, minpre0=-f0;
	maxCur=f0, minCur=-f0, maxpreCur=f0, minpreCur=-f0;
	maxRes=f0, minRes=-f0, maxpreRes=f0, minpreRes=-f0, diffRes=diffpreRes=2*f0, meanRes=sumRes=maxabsRes=0.0;
	cntCur=cnt0=cnt00;
	cntpreCur = (pre00<=0||pre00>=cnt00) ? cnt00/2 : cnt0-pre00;
	flag = 0;
}

void CMaxMin::Restart(void)
{
	Init(cnt0);
}

int CMaxMin::Update(float f)
{
	flag=0;
	if(maxCur<f) maxCur=f; else if(minCur>f) minCur=f;
	if(maxpreCur<f) maxpreCur=f; else if(minpreCur>f) minpreCur=f;
	sumRes += f;
	if(--cntCur<=0) {
		maxRes=maxCur; minRes=minCur; maxCur=minCur=f; diffRes=maxRes-minRes; meanRes=sumRes/cnt0; cntCur=cnt0; flag=1;
		maxabsRes = minRes<-maxRes ? -minRes : maxRes;
		sumRes = 0.0;
	}
	if(--cntpreCur<=0) {
		maxpreRes=maxpreCur; minpreRes=minpreCur; maxpreCur=minpreCur=f; diffpreRes=maxpreRes-minpreRes; cntpreCur=cnt0; flag=-1;
	}
	return flag;
}

//***************************  class CMaxMinn  *********************************/
CMaxMinn::CMaxMinn(int n0, int cnt00, int pre00, float f0)
{
	Init(n0, cnt00, pre00, f0);
}

void CMaxMinn::Init(int n0, int cnt00, int pre00, float f0)
{
	n = n0;
	CMaxMin mm0(cnt00, pre00, f0);
	for(int i=0; i<n; i++)	mm[i] = mm0;
	flag = 0;
}

void CMaxMinn::Restart(void)
{
	for(int i=0; i<n; i++) mm[i].Restart();
	flag = 0;
}

int CMaxMinn::Update(float f, ...)
{
	CMaxMin *pm=&mm[0];
	va_list vl;
	va_start(vl, f);
	for(int i=0; i<n; i++,pm++)
	{
		pm->Update(f);
		f = va_arg(vl, float);
	}
	va_end(vl);
	return flag = mm[0].flag;
}

int CMaxMinn::Update(const CVect3 &v1)
{
	mm[0].Update((float)v1.i); mm[1].Update((float)v1.j); mm[2].Update((float)v1.k);
	return flag = mm[0].flag;
}

int CMaxMinn::Update(const CVect3 &v1, const CVect3 &v2)
{
	mm[0].Update((float)v1.i); mm[1].Update((float)v1.j); mm[2].Update((float)v1.k);
	mm[3].Update((float)v2.i); mm[4].Update((float)v2.j); mm[5].Update((float)v2.k);
	return flag = mm[0].flag;
}

int CMaxMinn::Update(const CVect3 &v1, const CVect3 &v2, const CVect3 &v3)
{
	mm[0].Update((float)v1.i); mm[1].Update((float)v1.j); mm[2].Update((float)v1.k);
	mm[3].Update((float)v2.i); mm[4].Update((float)v2.j); mm[5].Update((float)v2.k);
	mm[6].Update((float)v3.i); mm[7].Update((float)v3.j); mm[8].Update((float)v3.k);
	return flag = mm[0].flag;
}

int CMaxMinn::Update(const CVect3 &v1, const CVect3 &v2, const CVect3 &v3, const CVect3 &v4)
{
	mm[0].Update((float)v1.i); mm[1].Update((float)v1.j); mm[2].Update((float)v1.k);
	mm[3].Update((float)v2.i); mm[4].Update((float)v2.j); mm[5].Update((float)v2.k);
	mm[6].Update((float)v3.i); mm[7].Update((float)v3.j); mm[8].Update((float)v3.k);
	mm[9].Update((float)v4.i); mm[10].Update((float)v4.j); mm[11].Update((float)v4.k);
	return flag = mm[0].flag;
}

int CMaxMinn::Update(const CVect3 &v1, const CVect3 &v2, const CVect3 &v3, const CVect3 &v4, const CVect3 &v5)
{
	mm[0].Update((float)v1.i); mm[1].Update((float)v1.j); mm[2].Update((float)v1.k);
	mm[3].Update((float)v2.i); mm[4].Update((float)v2.j); mm[5].Update((float)v2.k);
	mm[6].Update((float)v3.i); mm[7].Update((float)v3.j); mm[8].Update((float)v3.k);
	mm[9].Update((float)v4.i); mm[10].Update((float)v4.j); mm[11].Update((float)v4.k);
	mm[12].Update((float)v5.i); mm[13].Update((float)v5.j); mm[14].Update((float)v5.k);
	return flag = mm[0].flag;
}

float CMaxMinn::ResFloat(int i, int minmeanmaxFlag)
{
	     if(minmeanmaxFlag==-1) return mm[i].minRes;    // min
	else if(minmeanmaxFlag==0 ) return mm[i].meanRes;   // mean
	else if(minmeanmaxFlag==1 ) return mm[i].maxRes;    // max
	else if(minmeanmaxFlag==2 ) return mm[i].maxabsRes; // maxabs
	else return 0.0f;
}

CVect3 CMaxMinn::ResVect3(int i, int minmeanmaxFlag)
{
	CMaxMin *pi0=&mm[i], *pi1=pi0+1, *pi2=pi0+2;
	     if(minmeanmaxFlag==-1) return CVect3(pi0->minRes,    pi1->minRes,    pi2->minRes);    // min
	else if(minmeanmaxFlag==0 ) return CVect3(pi0->meanRes,   pi1->meanRes,   pi2->meanRes);   // mean
	else if(minmeanmaxFlag==1 ) return CVect3(pi0->maxRes,    pi1->maxRes,    pi2->maxRes);    // max
	else if(minmeanmaxFlag==2 ) return CVect3(pi0->maxabsRes, pi1->maxabsRes, pi2->maxabsRes); // maxabs
	else if(minmeanmaxFlag==3 ) return CVect3(pi0->diffRes,   pi1->diffRes,   pi2->diffRes);   // diffRes
	else return O31;
}

//***************************  class CKalman  *********************************/
CKalman::CKalman(void)
{
}

CKalman::CKalman(int nq0, int nr0)
{
//	psinsassert(nq0<=MMD&&nr0<=MMD);
	if(!(nq0<=MMD&&nr0<=MMD)) { /*printf("\tMMD too small!\n");*/ exit(0); }
	Init(nq0, nr0);
}

void CKalman::Init(int nq0, int nr0)
{
	kftk = 0.0;
	nq = nq0; nr = nr0;
	Ft = Pk = CMat(nq,nq,0.0);
	Hk = CMat(nr,nq,0.0);  Fading = CMat(nr,nq,1.0); zfdafa = 0.1f;
	Qt = Pmin = Xk = CVect(nq,0.0);  Xmax = Pmax = CVect(nq,INF);  Pset = CVect(nq,-INF);
	Zk = Zk_1 = CVect(nr,0.0);  Rt = Rt0 = CVect(nr,INF); Rset = CVect(nr,-INF); rts = CVect(nr,1.0);  Zfd = CVect(nr,0.0); Zfd0 = Zmm0 = Zmax = CVect(nr,INF);
	innoPre = CVect(nr,0.0); innoDiffMax = CVect(nr,INF);
	RtTau = Rmax = CVect(nr,INF); measstop = measlost = Rmin = Rb = Rstop = CVect(nr,0.0); Rbeta = CVect(nr,1.0);
	SetRmaxcount(5);
	innoMax = CVect(nr,INF);
	SetInnoMaxcount(5);
	FBTau = FBMax = FBOne = FBOne1 = CVect(nq,INF); FBXk = FBTotal = CVect(nq,0.0);
	kfcount = measflag = measflaglog = 0;  SetMeasMask(nr0,3);
	Zmm.Init(nr0, 10);
}

void CKalman::SetRmmbt(double rmin, double rmax, double b, double tau)
{
	if(tau<INF/2)  RtTau = tau;
	if(b<INF/2)  Rb = b;
	if(rmax<INF/2)  Rmax = Rt * (rmax*rmax);
	Rmin = Rt * (rmin*rmin);
}

void CKalman::SetRmaxcount(int cnt)
{
	for(int i=0; i<nr; i++) { Rmaxcount[i]=0, Rmaxcount0[i]=cnt; }
}

void CKalman::SetInnoMaxcount(int cnt)
{
	for(int i=0; i<nr; i++) { innoMaxcount[i]=0, innoMaxcount0=cnt; }
}

void CKalman::SetZmm(int zi, int pi, double zmm0, int cnt)
{
	if(cnt>0) Zmm.mm[zi].Init(cnt);
	Zmmpk[zi] = pi;  Zmm0.dd[zi] = zmm0; 
}

void CKalman::SetZmmVn(const CVect3 &zmm0, int cnt)
{
	SetZmm(0, 3, zmm0.i, cnt);	SetZmm(1, 4, zmm0.j, cnt);	SetZmm(2, 5, zmm0.k, cnt);
}

void CKalman::SetZmmPos(const CVect3 &zmm0, int cnt)
{
	SetZmm(3, 6, zmm0.i, cnt);	SetZmm(4, 7, zmm0.j, cnt);	SetZmm(5, 8, zmm0.k, cnt);
}

void CKalman::TimeUpdate(double kfts0, int fback)
{
	CMat Fk;
	kftk += kfts0;  kfcount++;
	SetFt(nq);
	Fk = ++(Ft*kfts0);  // Fk = I+Ft*ts
	Xk = Fk * Xk;
	Pk = Fk*Pk*(~Fk);  Pk += Qt*kfts0;
	if(fback)  Feedback(nq, kfts0);
	for(int i=0; i<nr; i++) {
		measlost.dd[i] += kfts0;
		if(measstop.dd[i]>0.0f) measstop.dd[i] -= kfts0;
	}
}

void CKalman::SetStatMask(unsigned int mask2, int k2, unsigned int mask1, int k1, unsigned int mask0)
{
	int k;
	for(k=0; k<k1; k++) {  // mask0 for states [0 ~ k1)
		if(!(mask0&(0x01<<k)))      { Pk.ZeroRC(k); Pmin(k)=Qt(k)=Xk(k)=0.0; }
	}
	for(k=k1; k<k2; k++) { // mask1 for states [k1 ~ k2)
		if(!(mask1&(0x01<<(k-k1)))) { Pk.ZeroRC(k); Pmin(k)=Qt(k)=Xk(k)=0.0; }
	}
	for(k=k2; k<nq; k++) { // mask2 for states [k2 ~ nq)
		if(!(mask2&(0x01<<(k-k2)))) { Pk.ZeroRC(k); Pmin(k)=Qt(k)=Xk(k)=0.0; }
	}
}

void CKalman::SetMeasMask(unsigned int mask, int type)
{
	unsigned int m;
	if(type==1) measmask = mask;		// set mask 1
	else if(type==0) measmask &= ~mask;	// delete mask to 0
	else if(type==2) measmask |= mask;	// add mask 1
	else if(type==3) {					// set mask-LSB 1
		for(m=0; mask>0; mask--)  m |= 0x01<<(mask-1);
		SetMeasMask(m);
	}
}

void CKalman::SetMeasFlag(unsigned int flag, int type)
{
	if(type==1)
		measflag = (flag==0) ? 0 : (measflag|flag);  // add the flag bits to 1
	else if(type==0)
		measflag &= ~flag;  // set the flag bits to 0
}

void CKalman::SetMeasStop(unsigned int meas, double stop)
{
	for(int i=0; i<measstop.rc; i++)  {	// assert(rc<32)
		if( meas&(0x01<<i) && (measstop.dd[i]<stop||stop<=0.0) ) measstop.dd[i]=stop;
	}
//	measstop.SetBit(meas, stop);
}

void CKalman::SetRadptStop(unsigned int meas, double stop)
{
	Rstop.SetBit(meas, stop);
}

int CKalman::MeasUpdate(double fading)
{
	CVect Pxz, Kk, Hi;
	SetMeas();
	for(int i=0; i<nr; i++)
	{
		if(((measflag&measmask)&(0x01<<i)) && measstop.dd[i]>EPS)
		{
			Hi = Hk.GetRow(i);
			Pxz = Pk*(~Hi);
			double Pz0 = (Hi*Pxz)(0,0), r=Zk(i)-(Hi*Xk)(0,0);
			if(Rb.dd[i]>EPS)
				RAdaptive(i, r, Pz0);
			if(Zfd.dd[i]<INFp5)
				RPkFading(i);
			double Pzz = Pz0+Rt.dd[i]/rts.dd[i];
			Kk = Pxz*(1.0/Pzz);
			Xk += Kk*r;
			Pk -= Kk*(~Pxz);
			measlost.dd[i] = 0.0;
		}
	}
	if(fading>1.0) Pk *= fading;
	XPConstrain();
	symmetry(Pk);
	int measres = measflag&measmask;
	measflaglog |= measres;
	SetMeasFlag(0);
	return measres;
}

void MeasUD(double U[], double D[], const double H[], double R, double K[], int n)  // Ref: my book P291
{
	int i, j, s;
	double f[MMD], g[MMD], DUHR[MMD], afa=R;
	for(i=0; i<n; i++) {
		for(f[i]=H[i],j=0; j<i; j++) f[i] += H[j]*U[j*n+i];  // U: upper triangle 
		g[i] = D[i]*f[i];
		afa += f[i]*g[i];
	}
	for(j=n-1; j>=0; j--) {
		double afa0=afa-f[j]*g[j], lambda=-f[j]/afa0;
		DUHR[j] = H[j];
		D[j] *= afa0/afa;  afa=afa0;
		for(i=j-1; i>=0; i--) {
			U[i*n+j] += lambda*g[i]; 
			for(s=i+1; s<j; s++) U[i*n+j] += lambda*U[i*n+s]*g[s];
			DUHR[j] += H[i]*U[i*n+j];  // H*U = U^T*H^T
		}
		DUHR[j] *= D[j]/R;
	}
	for(i=0; i<n; i++) {
		for(K[i]=DUHR[i],j=i+1; j<n; j++) K[i] += U[i*n+j]*DUHR[j];  // K=U*D*U^T*H^T/R
	}
}

int CKalman::RAdaptive(int i, double r, double Pr)
{
	double rr=r*r-Pr;
	if(Rb.dd[i]>1.0)	{  // s^2=Rb.dd[i], for heavy-tailed noise
		Rt.dd[i] = rr/Rb.dd[i];  // rho^2/s^2;
		if(Rt.dd[i]<Rmin.dd[i]) Rt.dd[i]=Rmin.dd[i];
		return 1; //adptOK=1;
	}
	if(rr<Rmin.dd[i])	rr = Rmin.dd[i];
	if(rr>Rmax.dd[i])	{ Rt.dd[i]=Rmax.dd[i]; Rmaxcount[i]++; }  
	else				{ Rt.dd[i]=(1.0-Rbeta.dd[i])*Rt.dd[i]+Rbeta.dd[i]*rr; Rmaxcount[i]=0; }
	Rbeta.dd[i] = Rbeta.dd[i]/(Rbeta.dd[i]+Rb.dd[i]);   // beta = beta / (beta+b)
	int adptOK = (Rmaxcount[i]==0||Rmaxcount[i]>Rmaxcount0[i]) ? 1: 0;
	return adptOK;
}

void CKalman::RPkFading(int i)
{
	Zfd.dd[i] = Zfd.dd[i]*(1.0f-zfdafa) + Zk.dd[i]*zfdafa;
	if(Zfd.dd[i]>Zfd0.dd[i] || Zfd.dd[i]<-Zfd0.dd[i])
		DVMDVafa(Fading.GetRow(i), Pk);
}

void CKalman::ZmmPkSet(int i)
{
	CMaxMin *pmm=&Zmm.mm[i];
	pmm->Update((float)Zk.dd[i]);
	if(pmm->flag) {  // if conitnously big abs(Zk[i]), enlarge Pk[i,i]
		if( pmm->minRes>Zmm0.dd[i] || pmm->maxRes<-Zmm0.dd[i] )
			Pset.dd[Zmmpk[i]] = pmm->meanRes*pmm->meanRes;
	}
}

void CKalman::XPConstrain(void)
{
	int i=0, nq1=nq+1;
	for(double *px=Xk.dd,*pxmax=Xmax.dd,*p=Pk.dd,*pmin=Pmin.dd,*pminEnd=&Pmin.dd[nq],*pmax=Pmax.dd,*pset=Pset.dd;
		pmin<pminEnd; px++,pxmax++,p+=nq1,pmin++,pmax++,pset++)
	{
		if(*px>*pxmax)		// Xk constrain
		{
			*px = *pxmax;
		}
		else if(*px<-*pxmax)
		{
			*px = -*pxmax;
		}
		if(*p<*pmin)		// Pk constrain
		{
			*p = *pmin;
		}
		else if(*p>*pmax)
		{
			double sqf=sqrt(*pmax/(*p))*0.9;
			for(double *prow=&Pk.dd[i*Pk.clm],*prowEnd=prow+nq,*pclm=&Pk.dd[i]; prow<prowEnd; prow++,pclm+=nq)
			{
				*prow *= sqf;
				*pclm = *prow;
			}
			Pk.dd[i*Pk.clm+i] *= sqf;  //20200303
			break;
		}
		if(*pset>0.0)	// Pk set
		{
			if(*p<*pset)
			{
				*p = *pset;
			}
			else if(*p>*pset)
			{
				double sqf=sqrt(*pset/(*p));
				for(double *prow=&Pk.dd[i*Pk.clm],*prowEnd=prow+nq,*pclm=&Pk.dd[i]; prow<prowEnd; prow++,pclm+=nq)
				{
					*prow *= sqf;
					*pclm *= sqf;
				}
			}
			*pset = -1.0;
		}
		i++;
	}
}

void CKalman::PmaxPminCheck(void)
{
	for(double *p=Pk.dd,*pmax=Pmax.dd,*pmin=Pmin.dd,*pminEnd=&Pmin.dd[nq]; pmin<pminEnd; p+=nq+1,pmax++,pmin++)
	{
		if(*p>*pmax) *pmax = *p*10.0;
		if(*p<EPS)	 *pmin = 0.0;  else if(*p<*pmin)  *pmin = *p/2.0;
	}
}

void CKalman::Feedback(int nnq, double fbts)
{
	double *pTau=FBTau.dd, *pTotal=FBTotal.dd, *pMax=FBMax.dd, *pOne=FBOne.dd, *pOne1=FBOne1.dd, *pXk=FBXk.dd, *p=Xk.dd;
	for(int i=0; i<nq; i++, pTau++,pTotal++,pMax++,pOne++,pOne1++,pXk++,p++)
	{
		if(*pTau<INFp5)
		{
			if(*p>*pOne1 || *p<-*pOne1) {  // max feedback/one step
				*pXk=*p;
			}
			else {
				double afa = fbts<*pTau ? fbts/(*pTau) : 1.0;
				*pXk = *p*afa;
				if(*pXk>*pOne) *pXk=*pOne; else if(*pXk<-*pOne) *pXk=-*pOne;  // min feedback/one step
			}
			if(*pMax<INFp5)
			{
				if(*pTotal+*pXk>*pMax)			*pXk = *pMax-*pTotal;
				else if(*pTotal+*pXk<-*pMax)	*pXk = -*pMax-*pTotal;
			}
			*p -= *pXk;
			*pTotal += *pXk;
		}
		else
		{
			*pXk = 0.0;
		}
	}
}

void CKalman::FeedbackAll(void)
{
	CVect oldFB=FBTau, oldFBOne=FBOne, oldFBMax=FBMax;
	FBTau = 0.0;  FBOne = INF;  FBMax = INF;
	Feedback(nq, 1.0);
	FBTau = oldFB;  FBOne = oldFBOne;  FBMax = oldFBMax;
	measflag = 0;
}

void CKalman::RtFading(int i, double fdts)
{
	double Taui=RtTau.dd[i], Rti=Rt.dd[i], Rmaxi=Rmax.dd[i];
	if(measlost.dd[i]>3.0 && Taui<INFp5 && Rb.dd[i]>0.0 && Rti<Rmaxi)
	{
		double afa = fdts<Taui ? fdts/Taui : 1.0;
		Rti += 2*sqrt(Rmaxi*Rti)*afa;
		Rt.dd[i] = Rti;
	}
}

void fusion(double *x1, double *p1, const double *x2, const double *p2, int n, double *xf, double *pf)
{
	if(xf==NULL) { xf=x1, pf=p1; }
	double *x10, *xf0;
	CVect3 att1;
	if(n<100) {  // n<100 for att(1:3), n>100 for no_att(1:3)
		x10 = x1, xf0 = xf;
		att1 = *(CVect3*)x1; 
		*(CVect3*)x2 = qq2phi(a2qua(*(CVect3*)x2),a2qua(att1));
		*(CVect3*)x1 = O31;
	}
	int j;
	for(j=(n>100)?100:0; j<n; j++,x1++,p1++,x2++,p2++,xf++,pf++)
	{
		double p1p2 = *p1+*p2;
		*xf = (*p1**x2 + *p2**x1)/p1p2; 
		*pf = *p1**p2/p1p2;
	}
	if(j<100) {
		*(CVect3*)xf0 = q2att(a2qua(att1)+*(CVect3*)xf0);
		if(xf0!=x10) *(CVect3*)x10 = att1; 
	}
}

void fusion(CVect3 &x1, CVect3 &p1, const CVect3 x2, const CVect3 p2)
{
	fusion(&x1.i, &p1.i, &x2.i, &p2.i, 103);
}

void fusion(CVect3 &x1, CVect3 &p1, const CVect3 x2, const CVect3 p2,
			CVect3 &xf, CVect3 &pf)
{
	fusion(&x1.i, &p1.i, &x2.i, &p2.i, 103, &xf.i, &pf.i);
}


//***************************  class CSINSTDKF  *********************************/
CSINSTDKF::CSINSTDKF(void)
{
}

CSINSTDKF::CSINSTDKF(int nq0, int nr0)
{
	CKalman::Init(nq0, nr0);
	Kkp2y = Kkv2y = 1.0;
}

void CSINSTDKF::Init(const CSINS &sins0)
{
	sins = sins0;  kftk = sins.tk;
	Fk = eye(nq);  Pk1 = CMat(nq,nq, 0.0);
	Pxz = Qk = Kk = tmeas = CVect(nq, 0.0);
	meantdts = 1.0; tdts = 0.0;
	maxStep = 2*(nq+nr)+3;
	TDReset();
	curOutStep = 0; maxOutStep = 1;
	timcnt0 = timcnt1 = 0, timcntmax = 100;  burden = 0.0;
	cststt = nq;  for(int k=0; k<nq; k++) hi1[k]=-1;
}

void CSINSTDKF::TDReset(void)
{
	iter = -2;
	ifn = 0;	meanfn = O31;
	SetMeasFlag(0);
}

double CSINSTDKF::Innovationi(int row)
{
	double hx=0.0;
	for(double *ph=&Hk.dd[row*nq], *px=&Xk.dd[0], *pxEnd=&Xk.dd[Xk.rc]; px<pxEnd; ph++,px++)  { hx += (*ph)*(*px); }
	return (Zk.dd[row]-hx);
}

int CSINSTDKF::TDUpdate(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts, int nStep)
{
	sins.Update(pwm, pvm, nSamples, ts);
	if(++curOutStep>=maxOutStep) { RTOutput(), curOutStep=0; }
	Feedback(nq, sins.nts);
	for(int j=0; j<nr; j++) {
		measlost.dd[j] += sins.nts;
		if(Rstop.dd[j]>0.0) Rstop.dd[j] -= sins.nts;
		if(measstop.dd[j]>0.0) measstop.dd[j] -= sins.nts;
	}

	measRes = 0;

	if(nStep<=0||nStep>=maxStep) { nStep=maxStep; }
	tdStep = nStep;

	tdts += sins.nts; kftk = sins.tk;  kfcount++;
//	meanfn = meanfn+sins.fn; ifn++;
	VADDE(meanfn, sins.fn); ifn++;
	for(int i=0; i<nStep; i++)
	{
		if(iter==-2)			// -2: set measurements
		{
			if(ifn==0)	break;
			CVect3 fn=sins.fn, an=sins.an;
			sins.fn = meanfn*(1.0/ifn); meanfn = O31; ifn = 0;
			sins.an = sins.fn+sins.eth.gcc; 
			SetFt(nq);
			SetMeas(); SetHk(nq); sins.fn = fn; sins.an = an;
		}
		else if(iter==-1)			// -1: discrete
		{
//			Fk = ++(Ft*tdts); // Fk = I+Ft*ts
			double *pFk,*pFt,*pEnd;
			for(pFk=Fk.dd,pFt=Ft.dd,pEnd=&Fk.dd[cststt*Fk.clm]; pFk<pEnd; pFk++,pFt++)  *pFk=*pFt*tdts;
			for(pFk=Fk.dd; pFk<pEnd; pFk+=Fk.clm+1)  *pFk+=1.0;
//			Xk = Fk*Xk;
			pFk=Fk.dd, pEnd=&Xk.dd[Xk.rc];  int jj;
			for(jj=0; jj<cststt; jj++)  {
				double f=0.0;
				for(double *pX=Xk.dd; pX<pEnd; pFk++,pX++)  f += *pFk * *pX;
				Pxz.dd[jj] = f;  // Pxz just for store data
			}
			for(jj=0; jj<cststt; jj++)  Xk.dd[jj] = Pxz.dd[jj];
#ifndef PSINS_FAST_CALCULATION
			Qk = Qt*tdts;
#else
			mul(Qk, Qt, tdts);
#endif
//			RtFading(tdts);
			meantdts = tdts; tdts = 0.0;
		}
		else if(iter<nq)		// 0 -> (nq-1): Fk*Pk
		{
			int row=iter;
			RowMul(Pk1, Fk, Pk, row, cststt);
		}
		else if(iter<2*nq)		// nq -> (2*nq-1): Fk*Pk*Fk+Qk
		{
			int row=iter-nq;
			RowMulT(Pk, Pk1, Fk, row, cststt);
			Pk.dd[nq*row+row] += Qk.dd[row];
//			if(row==nq-1) {	Pk += Qk; }
		}
		else if(iter<2*(nq+nr))	// (2*nq) -> (2*(nq+nr)-1): sequential measurement updating
		{
			int row=(iter-2*Ft.row)/2;
			int flag = (measflag&measmask)&(0x01<<row);
			if(flag)
			{
//				if((iter-2*Ft.row)%2==0)
				if(iter%2==0)
				{
					Hk.GetRow(Hi, row);  int hi=hi1[row];
					if(hi>=0) {
						Pk.GetClm(Pxz, hi);
						Pz0 = Pxz.dd[hi];
						innovation = Zk.dd[row]-Xk.dd[hi];  Zk_1.dd[row] = Zk.dd[row];
					}
					else {
#ifndef PSINS_FAST_CALCULATION
						Pxz = Pk*(~Hi);
#else
						mul(Pxz, Pk, Hi);
#endif
						Pz0 = dot(Hi,Pxz);
						innovation = Zk.dd[row]-dot(Hi,Xk);  Zk_1.dd[row] = Zk.dd[row];
					}
					double innoDiff = innovation - innoPre.dd[row];  innoPre.dd[row] = innovation;
					int *pinnoMaxcount=&innoMaxcount[row];
					if(*pinnoMaxcount>0) (*pinnoMaxcount)--;
					if(innovation<-innoMax.dd[row] || innovation>innoMax.dd[row])
					{
						if(*pinnoMaxcount<2*innoMaxcount0) *pinnoMaxcount += 2;
					}
					if( (*pinnoMaxcount==0||*pinnoMaxcount>innoMaxcount0) 
						&& (innovation>-Zmax.dd[row]&&innovation<Zmax.dd[row])
						&& (innoDiff>-innoDiffMax.dd[row]&&innoDiff<innoDiffMax.dd[row]) )
					{
						adptOKi = 1;
						if(Rb.dd[row]>0.0 && Rstop.dd[row]<=0.0) {
							adptOKi=RAdaptive(row, innovation, Pz0);
						}
						if(row==5)
							int a=1;
						if(Rset.dd[row]>0.0) {
							Rt.dd[row]=Rset.dd[row];  Rset.dd[row]=-1.0;  adptOKi=1;
						}
						double Pzz = Pz0 + Rt.dd[row]/rts.dd[row];
#ifndef PSINS_FAST_CALCULATION
						Kk = Pxz*(1.0/Pzz);
#else
						mul(Kk, Pxz, (1.0/Pzz));
#endif
					}
					else
					{
						adptOKi = 0;
					}
				}
				else
				{
					measflag ^= flag;
					if(adptOKi && measstop.dd[row]<EPS)
					{
						measRes |= flag;
#ifndef PSINS_FAST_CALCULATION
						Pk -= Kk*(~Pxz);
#else
						double *pPk=Pk.dd, *pKk=Kk.dd, *pKkEnd=&Kk.dd[Kk.row], *pPxzEnd=&Pxz.dd[Pxz.row];
						for(; pKk<pKkEnd; pKk++) {
							for(double *pPxz=Pxz.dd; pPxz<pPxzEnd; pPxz++)
								*pPk++ -= *pKk * *pPxz;
						}
#endif
						if(flag&030) Kk.dd[2]*=Kkp2y;  // disable pos2yaw
						else if(flag&003) Kk.dd[2]*=Kkv2y;
#ifndef PSINS_FAST_CALCULATION
						Xk += Kk*innovation;
#else
						for(double *pXk=Xk.dd,*pXkEnd=&Xk.dd[Xk.row],*pKk1=Kk.dd; pXk<pXkEnd; pXk++,pKk1++) *pXk += *pKk1 * innovation;
#endif
						measlost.dd[row] = 0.0;
					}
					if(Zfd0.dd[row]<INFp5)
					{
						RPkFading(row);
					}
					if(Zmm0.dd[row]<INFp5)
					{
						ZmmPkSet(row);
					}
				}
			}
			else
			{
				nStep++;
			}
			if(iter%2==0 && Rstop.dd[row]<EPS)
				RtFading(row, meantdts);
		}
		else if(iter==2*(nq+nr))	// 2*(nq+nr): Xk,Pk constrain & symmetry
		{
			XPConstrain();
			symmetry(Pk);
		}
		else if(iter>=2*(nq+nr)+1)	// 2*(nq+nr)+1: Miscellanous
		{
			Miscellanous();
			iter = -3;
		}
		iter++;
	}
	SecretAttitude();

	measflaglog |= measRes;
	return measRes;
}

void CSINSTDKF::MeasUpdate(const CVect &Hi, double Ri, double Zi)
{
	if(iter>=0 && iter<2*nq) return;  // !
	Pxz = Pk*(~Hi);
	Pz0 = (Hi*Pxz)(0,0);
	innovation = Zi-(Hi*Xk)(0,0);
	double Pzz = Pz0 + Ri;
	Kk = Pxz*(1.0/Pzz);
	Xk += Kk*innovation;
	Pk -= Kk*(~Pxz);
}

void CSINSTDKF::MarkovGyro(const CVect3 &tauG, const CVect3 &sRG, int stateeb)
{
	sins.SetTauGA(tauG, O31);
	*(CVect3*)&Qt.dd[stateeb] = MKQt(sRG, sins.tauGyro);
}

void CSINSTDKF::MarkovAcc(const CVect3 &tauA, const CVect3 &sRA, int statedb)
{
	sins.SetTauGA(O31, tauA);
	*(CVect3*)&Qt.dd[statedb] = MKQt(sRA, sins.tauAcc);
}

void CSINSTDKF::SetYaw(double yaw, int statephi, int statedvn)
{
	CQuat qnn = a2qua(0,0,diffYaw(yaw,sins.att.k));
	sins.qnb = qnn*sins.qnb;  sins.Cnb = q2mat(sins.qnb);  sins.Cbn = ~sins.Cnb;  sins.att = m2att(sins.Cnb);
	sins.vn = qnn*sins.vn;
	*(CVect3*)&Xk.dd[statephi] = qnn**(CVect3*)&Xk.dd[statephi];
	*(CVect3*)&Xk.dd[statedvn] = qnn**(CVect3*)&Xk.dd[statedvn];
	Pk = diag(diag(Pk));
/*	CMat3 Cnn=q2mat(qnn);
	CMat Cnn15(15,15,0.0);
	Cnn15.SetMat3(0,0, Cnn); Cnn15.SetMat3(3,3, Cnn);Cnn15.SetMat3(6,6, rcijk(Cnn,102)); 
	Cnn15.SetMat3(9,9, Cnn); Cnn15.SetMat3(12,12, Cnn);
	Pk = (Cnn15)*Pk*(~Cnn15);*/
	TDReset();
}

void CSINSTDKF::PSetVertCh(double sph, double spv, double spd)
{
	sph = sph*sph;  spv = spv*spv;  spd = spd*spd;
	if(sph>Pk.dd[nq*8+8])   Pset.dd[8]  = sph;
	if(spv>Pk.dd[nq*5+5])   Pset.dd[5]  = spv;
	if(spd>Pk.dd[nq*14+14]) Pset.dd[14] = spd;
}

double CSINSTDKF::SetCalcuBurden(unsigned int timcnt, int itype)
{
	double res=0.0;
	if(itype==-1) {
		timcntmax = timcnt;
	}
	else if(itype==0) {
		timcnt0 = timcnt;
	}
	else {
		timcnt1 = timcnt;
		if(timcntmax<timcnt1) timcntmax=timcnt1;
		if(timcnt1<timcnt0) timcnt1+=timcntmax;
		res = (double)(timcnt1-timcnt0)/timcntmax;
		if(res>0.99) res=0.99;
		burden = burden<res ? res : burden-0.01;
	}
	return (iter+100)+res;
}

//***************************  class CSINSGNSS  *********************************/
CSINSGNSS::CSINSGNSS(void)
{
}

CSINSGNSS::CSINSGNSS(int nq0, int nr0, double ts, int yawHkRow0):CSINSTDKF(nq0, nr0)
{
	deal(Xk.dd, &pphi,0, &pdvn,3, &pdpos,6, &peb,9, &pdb,12, &plvr,15, &pdT,18, 
		&pdkg1,19, &pdkg2,22, &pdkg3,25, &pdka1,28, &pdka23,31, NULL);
	deal(Xk.dd, &pdkgzz,19, &pdkgz,19, &pdkaii,22, &pddbz,nq0==23?22:25, NULL);
	navStatus = 0;
	posGNSSdelay = vnGNSSdelay = yawGNSSdelay = dtGNSSdelay = dyawGNSS = -0.0f;
	kfts = ts;	gnssLost = &measlost.dd[3];
	lvGNSS = O31;
	Hk(0,3) = Hk(1,4) = Hk(2,5) = 1.0;		// dvn
	Hk(3,6) = Hk(4,7) = Hk(5,8) = 1.0;		// dpos
	yawHkRow = yawHkRow0;
	if(yawHkRow>=6) Hk(yawHkRow,2) = 1.0;	// dyaw
	SetMeasMask(077);
	SetMeasStop(0xffffffff, 1.0);
}

void CSINSGNSS::Init(const CSINS &sins0, int grade)
{
	CSINSTDKF::Init(sins0);
	sins.lever(-lvGNSS, &sins.pos);   // sins0.pos is GNSS pos
	avpi.Init(sins, kfts, 0, 0);
	if(grade==0) {  // inertial grade
		Pmax.Set2(fDEG3(1.0),  fXXX(100.0),  fdPOS(1.0e6), fDPH3(0.5),  fMG3(1.0),
			fXXX(100.0), 0.5, fdKGA15(1000.0,900.0,100.0,100.0));
		Pmin.Set2(fPHI(0.1,1.0),  fXXX(0.001),  fdPOS(.01),  fDPH3(0.001),  fUG3(10.0),
			fXXX(0.001), 0.0001, fdKGA15(1.0,1.0,1.0,1.0));
		Pk.SetDiag2(fPHI(60,600),  fXXX(1.0),  fdPOS(100.0),  fDPH3(0.1),  fMG3(1.0),
			fXXX(1.0),  0.01,  fdKGA15(100.0,90.0,10.0,10.0));
		Qt.Set2(fDPSH3(0.001),  fUGPSHZ3(1.0),  fOOO,  fOO6,
			fOOO, 0.0,  fOO9,  fOO6);
		//MarkovGyro(I31*1000.0, I31*0.01*DPH);  MarkovAcc(I31*1000.0, I31*1.0*UG);
		Xmax.Set(fINF9,  fDPH3(0.5),  fMG3(1.0),
			fXXX(10.0), 0.5,  fdKGA15(1000.0,900.0,1000.0,900.0));
		Rt.Set2(fXXZ(0.5,1.0),   fdLLH(10.0,30.0));  Rt0 = Rt;
		Rmax = Rt*100;  Rmin = Rt*0.01;  Rb = 0.6f;
		innoMax.Set(fXXX(1.0), fdPOS(30.0));  SetInnoMaxcount(5);
		Zmax.Set(fXXX(10.0), fdPOS(1.0e6));
		innoDiffMax.Set(fXXZ(3.0,2.0), fdLLH(100.0,10.0));
		FBOne1.Set(fPHI(1,1), fXXX(0.1), fdLLH(1,3), fDPH3(0.01), fUG3(10), fXXX(0.1), 0.01, fINF9,  fINF6);
		FBTau.Set(fXX9(0.1),  fXX6(1.0),  fINF3, INF,  fINF9,  fINF6);
	}
	else if(grade==1) {  // MEMS grade
		Pmax.Set2(fDEG3(50.0),  fXXX(100.0),  fdPOS(1.0e6), fDPH3(5000.0),  fMG3(30.0),
			fXXX(100.0), 0.5, fdKGA15(1000.0,900.0,100.0,100.0));
		Pmin.Set2(fPHI(1,1),  fXXX(0.0001),  fdPOS(.001),  fDPH3(0.001),  fUG3(1.0),
			fXXX(0.001), 0.0001, fdKGA15(1.0,1.0,1.0,1.0));
		Pk.SetDiag2(fPHI(600,600),  fXXX(1.0),  fdPOS(100.0),  fDPH3(1000.0),  fMG3(10.0),
			fXXX(1.0),  0.01,  fdKGA15(1000.0,90.0,10.0,10.0));
		Qt.Set2(fDPSH3(1.1),  fUGPSHZ3(10.0),  fOOO,  fOO6,
			fOOO, 0.0,  fOO9,  fOO6);
		Xmax.Set(fINF9,  fDPH3(3600.0),  fMG3(50.0),
			fXXX(10.0), 0.5,  fdKGA15(1000.0,900.0,1000.0,900.0));
		Rt.Set2(fXXZ(0.5,1.0),   fdLLH(10.0,30.0));  Rt0 = Rt;
		Rmax = Rt*100;  Rmin = Rt*0.01;  Rb = 0.6f;
		FBTau.Set(fXX9(0.1),  fXX6(1.0),  fINF3, INF,  fINF9,  fINF6);
	}
}

void CSINSGNSS::SetFt(int nnq)
{
	sins.etm();
//	Ft.SetMat3(0,0,sins.Maa), Ft.SetMat3(0,3,sins.Mav), Ft.SetMat3(0,6,sins.Map), Ft.SetMat3(0,9,-sins.Cnb); 
//	Ft.SetMat3(3,0,sins.Mva), Ft.SetMat3(3,3,sins.Mvv), Ft.SetMat3(3,6,sins.Mvp), Ft.SetMat3(3,12,sins.Cnb); 
//						NULL, Ft.SetMat3(6,3,sins.Mpv), Ft.SetMat3(6,6,sins.Mpp);
	Ft.SetMat3(0,0,sins.Maa,sins.Mav,sins.Map), Ft.SetMat3(0,9,-sins.Cnb); 
	Ft.SetMat3(3,0,sins.Mva,sins.Mvv,sins.Mvp), Ft.SetMat3(3,12,sins.Cnb); 
	Ft.SetMat3(6,3,         sins.Mpv,sins.Mpp);
	Ft.SetDiagVect3( 9, 9, sins._betaGyro);
	Ft.SetDiagVect3(12,12, sins._betaAcc);  // 0-14 phi,dvn,dpos,eb,db
	if(nnq==15) return;
	if(nnq==16) {
		Ft(2,15) = -sins.wib.k*sins.Cnb.e22;  // 15 dKGzz
	}
	if(nnq>=18) NULL;						// 15-17 lever
	if(nnq>=19) NULL;						// 18 dt
	if(nnq==20) {
		Ft(2,19) = -sins.wib.k*sins.Cnb.e22;  // 19 dKGzz
	}
	else if(nnq==22) {
		CMat3 Cwz=-sins.wib.k*sins.Cnb;
		Ft.SetMat3(0,19, Cwz);				// 19-21 dKG*z
	}
	else if(nnq==23) {
		CMat3 Cwz=-sins.wib.k*sins.Cnb;
		Ft.SetMat3(0,19, Cwz);				// 19-21 dKG*z
		Ft(14,22)=1.0;						// 22 ddbz
	}
	else if(nnq==25) {
		CMat3 Cw(-sins.wib.i*sins.Cnb.e00,-sins.wib.j*sins.Cnb.e01,-sins.wib.k*sins.Cnb.e02,
			     -sins.wib.i*sins.Cnb.e10,-sins.wib.j*sins.Cnb.e11,-sins.wib.k*sins.Cnb.e12,
				 -sins.wib.i*sins.Cnb.e20,-sins.wib.j*sins.Cnb.e21,-sins.wib.k*sins.Cnb.e22 );
		Ft.SetMat3(0,19, Cw);				// 19-21 dKGii
		CMat3 Cf( sins.fb.i*sins.Cnb.e00, sins.fb.j*sins.Cnb.e01, sins.fb.k*sins.Cnb.e02,
			      sins.fb.i*sins.Cnb.e10, sins.fb.j*sins.Cnb.e11, sins.fb.k*sins.Cnb.e12,
				  sins.fb.i*sins.Cnb.e20, sins.fb.j*sins.Cnb.e21, sins.fb.k*sins.Cnb.e22 );
		Ft.SetMat3(3,22, Cf);				// 22-24 dKAii
	}
	else if(nnq==26) {
		CMat3 Cwz=-sins.wib.k*sins.Cnb;
		Ft.SetMat3(0,19, Cwz);				// 19-21 dKG*z
		CMat3 Cf( sins.fb.i*sins.Cnb.e00, sins.fb.j*sins.Cnb.e01, sins.fb.k*sins.Cnb.e02,
			      sins.fb.i*sins.Cnb.e10, sins.fb.j*sins.Cnb.e11, sins.fb.k*sins.Cnb.e12,
				  sins.fb.i*sins.Cnb.e20, sins.fb.j*sins.Cnb.e21, sins.fb.k*sins.Cnb.e22 );
		Ft.SetMat3(3,22, Cf);				// 22-24 dKAii
		Ft(14,25)=1.0;						// 25 ddbz
	}
	else if(nnq>=28) {
#ifndef PSINS_FAST_CALCULATION
		CMat3 Cwx=-sins.wib.i*sins.Cnb, Cwy=-sins.wib.j*sins.Cnb, Cwz=-sins.wib.k*sins.Cnb; 
#else
		double wi=-sins.wib.i, wj=-sins.wib.j, wk=-sins.wib.k;
		CMat3 Cwx, Cwy, Cwz; MMULf(Cwx,sins.Cnb,wi); MMULf(Cwy,sins.Cnb,wj); MMULf(Cwz,sins.Cnb,wk);
#endif
//		Ft.SetMat3(0,19, Cwx);  Ft.SetMat3(0,22, Cwy);  Ft.SetMat3(0,25, Cwz);  // 19-27 dKG
		Ft.SetMat3(0,19, Cwx, Cwy, Cwz);  // 19-27 dKG
	}
	if(nnq>=34) {
#ifndef PSINS_FAST_CALCULATION
		CMat3 Cfx= sins.fb.i *sins.Cnb, Cfy= sins.fb.j *sins.Cnb, Cfz= sins.fb.k *sins.Cnb;
#else
		CMat3 Cfx, Cfy, Cfz; MMULf(Cfx,sins.Cnb,sins.fb.i); MMULf(Cfy,sins.Cnb,sins.fb.j); MMULf(Cfz,sins.Cnb,sins.fb.k);
#endif
		Cfz.e00=Cfy.e01,	Cfz.e01=Cfy.e02; 
		Cfz.e10=Cfy.e11,	Cfz.e11=Cfy.e12; 
		Cfz.e20=Cfy.e21,	Cfz.e21=Cfy.e22;
//		Ft.SetMat3(3,28, Cfx);  Ft.SetMat3(3,31, Cfz);  // 28-33 dKA(xx,yx,zx, yy,zy, zz)
		Ft.SetMat3(3,28, Cfx, Cfz);  // 28-33 dKA(xx,yx,zx, yy,zy, zz)
	}
}

void CSINSGNSS::SetHk(int nnq)
{
	if(nnq>=18) {     // GNSS lever
		CMat3 CW=sins.Cnb*askew(sins.webbar), MC=sins.Mpv*sins.Cnb;
		Hk.SetMat3(0,15, -CW);
		Hk.SetMat3(3,15, -MC);
	}
	if(nnq>=19) {    // GNSS dt
		CVect3 MV=sins.Mpv*sins.vn;
//		Hk.SetClmVect3(0,18, -sins.anbar);
		Hk.SetClmVect3(3,18, -MV);
	}
}

void CSINSGNSS::Feedback(int nnq, double fbts)
{
	CKalman::Feedback(nq, fbts);
	for(int i=0; i<avpi.avpinum; i++) {
		avpi.vni[i] -= *(CVect3*)&FBXk.dd[ 3];  avpi.posi[i] -= *(CVect3*)&FBXk.dd[6];
	}
	sins.qnb -= *(CVect3*)&FBXk.dd[0];  sins.vn -= *(CVect3*)&FBXk.dd[ 3];  sins.pos -= *(CVect3*)&FBXk.dd[6];
	sins.eb  += *(CVect3*)&FBXk.dd[9];	sins.db += *(CVect3*)&FBXk.dd[12];  // 0-14 phi,dvn,dpos,eb,db
	if(nnq==15) return;
	if(nnq==16) {
		double IdKGzz = 1.0-FBXk.dd[15];
		sins.Kg.e20*=IdKGzz, sins.Kg.e21*=IdKGzz, sins.Kg.e22*=IdKGzz;  // 15 dKGzz
	}
	if(nnq>=18) {
		lvGNSS += *(CVect3*)&FBXk.dd[15];	// 15-17 lever
	}
	if(nnq>=19) {
		dtGNSSdelay += FBXk.dd[18];			// 18 dt
	}
	if(nnq==20) {
		double IdKGzz = 1.0-FBXk.dd[19];
		sins.Kg.e20*=IdKGzz, sins.Kg.e21*=IdKGzz, sins.Kg.e22*=IdKGzz;  // 19 dKGzz
	}
	else if(nnq==22) {
		CMat3 IdKGz(1.0,0.0,-FBXk.dd[19], 0.0,1.0,-FBXk.dd[20], 0.0,0.0,1.0-FBXk.dd[21]);
		sins.Kg = IdKGz*sins.Kg;		// 19-21 dKG*z
	}
	else if(nnq==23) {
		CMat3 IdKGz(1.0,0.0,-FBXk.dd[19], 0.0,1.0,-FBXk.dd[20], 0.0,0.0,1.0-FBXk.dd[21]);
		sins.Kg = IdKGz*sins.Kg;		// 19-21 dKG*z
		NULL;   // 22 ddbz
	}
	else if(nnq==25) {
		sins.Kg.e00*=1.0-FBXk.dd[19], sins.Kg.e11*=1.0-FBXk.dd[20], sins.Kg.e22*=1.0-FBXk.dd[21];	// 19-21 dKGii
		sins.Ka.e00*=1.0-FBXk.dd[22], sins.Ka.e11*=1.0-FBXk.dd[23], sins.Ka.e22*=1.0-FBXk.dd[24];	// 22-24 dKAii
	}
	else if(nnq==26) {
		CMat3 IdKGz(1.0,0.0,-FBXk.dd[19], 0.0,1.0,-FBXk.dd[20], 0.0,0.0,1.0-FBXk.dd[21]);
		sins.Kg = IdKGz*sins.Kg;		// 19-21 dKG*z
		sins.Ka.e00*=1.0-FBXk.dd[22], sins.Ka.e11*=1.0-FBXk.dd[23], sins.Ka.e22*=1.0-FBXk.dd[24];	// 22-24 dKAii
		NULL;   // 25 ddbz
	}
	else if(nnq>=28) {
		CMat3 IdKG = I33-(~(*(CMat3*)&FBXk.dd[19]));
		sins.Kg = IdKG*sins.Kg;			// 19-27 dKG
	}
	if(nnq>=34) {
		CMat3 IdKA(1.0-FBXk.dd[28],            0.0,           0.0,
			          -FBXk.dd[29],1.0-FBXk.dd[31],           0.0,
					  -FBXk.dd[30],   -FBXk.dd[32],1.0-FBXk.dd[33]);
		sins.Ka = IdKA*sins.Ka;			// 28-33 dKA
	}
}

void CSINSGNSS::SetMeasGNSS(const CVect3 &posgnss, const CVect3 &vngnss, double yawgnss)
{
	if(!IsZero(posgnss) && avpi.Interp(posGNSSdelay+dtGNSSdelay,0x4))
	{
		*(CVect3*)&Zk.dd[3] = avpi.pos - posgnss;
		SetMeasFlag(00070);
	}
	if(!IsZero(vngnss) && avpi.Interp(vnGNSSdelay+dtGNSSdelay,0x2))
	{
		*(CVect3*)&Zk.dd[0] = avpi.vn - vngnss;
		SetMeasFlag(00007);
	}
	if(!IsZero(yawgnss) && avpi.Interp(yawGNSSdelay+dtGNSSdelay,0x1))
	{
		Zk.dd[yawHkRow] = -diffYaw(avpi.att.k, yawgnss+dyawGNSS);
		SetMeasFlag(01<<yawHkRow);
	}
}

void CSINSGNSS::MeasGNSSZvStop(CVect3 &dvnth, double stop)  // Zv/dvn threshold value
{
	int meas=001;
	double *z=&Zk.dd[0], *v=&dvnth.i;
	for(int i=0; i<3; i++,z++,v++) { 
		if(*z>*v || *z<-(*v))	{ SetMeasStop(meas,stop); meas<<=1; }
	}
}

void CSINSGNSS::MeasGNSSZpStop(CVect3 &dposth, double stop)  // Zk/dpos threshold value
{
	int meas=010;
	double *z=&Zk.dd[3], *p=&dposth.i;
	for(int i=0; i<3; i++,z++,p++) { 
		if(*z>*p || *z<-(*p))	{ SetMeasStop(meas,stop); meas<<=1; }
	}
}

void CSINSGNSS::MeasGNSSZp2X(CVect3 &dposth)  // Zk/dpos threshold value
{
	double *z=&Zk.dd[3], *p=&dposth.i, *ps=&Pset.dd[6];
	for(int i=0; i<3; i++,z++,p++,ps++) {
		if(*z>*p || *z<-(*p))	{ *ps=100.0*(*z)*(*z);	}
	}
}

void CSINSGNSS::Leveling(void)
{
	CVect3 phi=*(CVect3*)&Xk.dd[0]; phi.k=0;
	sins.qnb -= phi;  Xk.dd[0]=Xk.dd[1]=0.0;
	sins.vn -= *(CVect3*)&Xk.dd[3];  *(CVect3*)&Xk.dd[3]=O31;
}

int CSINSGNSS::Update(const CVect3 *pwm, const CVect3 *pvm, int nn, double ts, int nSteps)
{
	int res=TDUpdate(pwm, pvm, nn, ts, nSteps);
	sins.lever(lvGNSS);
	avpi.Push(sins, 1);
	return res;
}

#ifdef PSINS_IO_FILE
void CSINSGNSS::operator<<(CFileRdWt &f)
{
	f<<sins.att<<sins.vn<<sins.pos<<sins.eb<<sins.db  // 1-15
		<<sins.vnL<<sins.posL<<lvGNSS   // 16-24
		<<dtGNSSdelay<<dyawGNSS <<kftk; // 25-27
}

void CSINSGNSS::LogXk(void)
{
	psinslog.MArray("phi", &Xk.dd[0], 3, 1, 1/glv.min, "arcmin");
	psinslog.MArray("dVn", &Xk.dd[3], 3, 1, 1.0, "m/s");
	CVect3 dpos=*(CVect3*)&Xk.dd[6]; dpos.i*=sins.eth.RMh, dpos.j*=sins.eth.clRNh;
	psinslog.MArray("dPos", &dpos.i, 3, 1, 1.0, "m");
	psinslog.MArray("eb", &Xk.dd[9], 3, 1, 1/glv.dph, "dph");
	psinslog.MArray("db", &Xk.dd[12], 3, 1, 1/glv.ug, "ug");
	if(nq==16) {
		psinslog.MArray("dKGzz", &Xk.dd[15], 1, 1, 1/glv.ppm, "ppm");
	}
	if(nq>=18) {
		psinslog.MArray("Lvr", &Xk.dd[15], 3, 1, 100, "cm");
	}
	if(nq>=19) {
		psinslog.MArray("dt", &Xk.dd[18], 1, 1, 1000, "ms");
	}
	if(nq==20) {
		psinslog.MArray("dKGzz", &Xk.dd[19], 1, 1, 1/glv.ppm, "ppm");
	}
	else if(nq==22) {
		CVect3 dKGz=*(CVect3*)&Xk.dd[19]; dKGz.i*=1/glv.sec,dKGz.j*=1/glv.sec,dKGz.k*=1/glv.ppm;
		psinslog.MArray("dKG*z", &Xk.dd[19], 3, 1, 1, "arcsec|ppm");
	}
	else if(nq>=28) {
		CMat3 dKG = ~(*(CMat3*)&Xk.dd[19]);
		dKG.e00*=1/glv.ppm*glv.sec, dKG.e11*=1/glv.ppm*glv.sec, dKG.e22*=1/glv.ppm*glv.sec;
		psinslog.MArray("dKG", &dKG.e00, 3, 3, 1/glv.sec, "arcsec|ppm");
	}
	if(nq>=34) {
		CMat3 dKA(Xk.dd[28], 0.0,       0.0,
			      Xk.dd[29], Xk.dd[31], 0.0,
				  Xk.dd[30], Xk.dd[32], Xk.dd[33]);
		dKA.e00*=1/glv.ppm*glv.sec, dKA.e11*=1/glv.ppm*glv.sec, dKA.e22*=1/glv.ppm*glv.sec;
		psinslog.MArray("dKA", &dKA.e00, 3, 3, 1/glv.sec, "arcsec|ppm");
	}
}
#endif

//***************************  class CSysClbt  *********************************/
CSysClbt::CSysClbt(const CVect3 &pos0, double g00, int ka2pn):CSINSTDKF(37, 3)
{
	iter = 0;
	wibStatic = 0.1*DPS;
	Hk.SetMat3(0, 3, I33);  hi1[0]=3,hi1[1]=4,hi1[2]=5;
	SetMeasMask(07);
	this->pos0 = pos0;
	ka2ORpn = ka2pn;
	Init(g00);
}

void CSysClbt::Init(double g00)
{
	CSINSTDKF::Init(CSINS(0.0));  sins.mvnT = 0.2;
	sins.isOpenloop = 1;
	sins.eth.Update(pos0);	gn = g00>9.0 ? CVect3(0,0,-g00) : sins.eth.gn;  sins.eth.pgn = &gn;
	sins.imu.SetKga();
	ka2ORpn ? sins.imu.SetKa2() : sins.imu.SetKapn();
	sins.imu.SetCba();
	sins.imu.SetLvtGA();
	// KF para
	Pmin.Set2(fPHI(0.01,0.1),  fdVEL(0.0001),  fDPH3(0.0001),  fUG3(1.0),
		fdKGA15(1.0,1.0,1.0,1.0), fdKapn3(0), fInLv6(0.001), 0.00001);
	Pk.SetDiag2(fPHI(10,60),  fdVEL(0.1),  fDPH3(0.1),  fUG3(100.0),
		fdKGA15(10000.0,3600.0,10000.0,3600.0), fdKapn3(0), fInLv6(1), 0.001);
	Qt.Set2(fDPSH3(0.001), fUGPSHZ3(1.0), fOO6,  fOO9,fOO6,  fOOO,fOO6,0.0);
	Rt.Set2(fXXZ(0.01,0.01));
	FBMax.Set(fPHI(60,60),  fdVEL(1000.0),  fDPH3(1.0),  fMG3(10.0),
		fdKGA15(10000.0,3600.0,10000.0,3600.0), fdKapn3(100), fInLv6(20), 0.01);
	if(ka2ORpn==1) {
		Pmin.Set2Vect3(27, CVect3(fdKa23(0)));
		Pk.SetMat3(27, 27, pow(diag(fdKa23(100)),2));
		FBMax.SetVect3(27, CVect3(fdKa23(100)));
	}
	FBTau = 1.0;
}

void CSysClbt::NextIter(const CQuat &qnb0)
{
	sins.qnb = qnb0;  sins.vn = O31;  sins.pos = pos0;
	TDReset();  sins.imu.Reset();
	FBTotal = 0.0;
	if(iter==1) {
		Pk.SetDiag2(fPHI(1,16),  fdVEL(0.1),  fDPH3(0.02),  fUG3(100.0),
			fdKGA15(1000.0,600.0,1000.0,600.0), fdKapn3(10), fInLv6(10), 0.01);
		if(ka2ORpn==1) Pk.SetMat3(27, 27, pow(diag(fdKa23(10)),2));
	}
	else if(iter>=2) {
		FBMax = FBTau = INF;
		Pk.SetDiag2(fPHI(1,6),  fdVEL(0.1),  fDPH3(0.01),  fUG3(100.0),
			fdKGA15(100.0,60.0,100.0,60.0), fdKapn3(100), fInLv6(0.1), 0.001);
		if(ka2ORpn==1) Pk.SetMat3(27, 27, pow(diag(fdKa23(100)),2));
	}
	iter++;
}

void CSysClbt::SetFt(int nnq)
{
	CVect3 fa=sins.fb*sins.imu.Cba; CMat3 Cna=sins.Cnb*sins.imu.iTCba;
	CMat3 CDf2 = sins.imu.pKa2 ? Cna*diag(pow(fa,2)) : Cna*diag(abs(fa));
//    CMat3 CDf2 = sins.imu.pKa2 ? sins.Cnb*diag(pow(sins.fb,2)) : sins.Cnb*diag(abs(sins.fb));
	CVect3 CwXf = sins.Cnb*(sins.wib*sins.fb);
	CMat3 Cfx = sins.fb.i*sins.Cnb, Cfy = sins.fb.j*sins.Cnb, Cfz = sins.fb.k*sins.Cnb;
	Cfz.e00=Cfy.e01,	Cfz.e01=Cfy.e02; 
	Cfz.e10=Cfy.e11,	Cfz.e11=Cfy.e12; 
	Cfz.e20=Cfy.e21,	Cfz.e21=Cfy.e22;
    //        0  3   6  9   12       15       18       21       24        26       27   30 33 36
    //states: fi dvn eb db  dKg(:,1) dKg(:,2) dKg(:,3) dKa(:,1) dKa(yz,2) dKa(z,3) dKa2 ry rz tGA
	Ft.SetMat3(0,0, -askew(sins.eth.wnie));	Ft.SetMat3(0,6, -sins.Cnb); 
	Ft.SetMat3(0,12, -sins.wib.i*sins.Cnb, -sins.wib.j*sins.Cnb, -sins.wib.k*sins.Cnb);
	Ft.SetMat3(3,0, askew(sins.fn)), Ft.SetMat3(3,9, sins.Cnb); 
	Ft.SetMat3(3,21, Cfx, Cfz, CDf2); 
	Ft.SetMat3(3,30, sins.Cnb*sins.imu.SSx, sins.Cnb*sins.imu.SSy);  Ft.SetClmVect3(3,36, CwXf);

	cststt = 6;
}

void CSysClbt::Feedback(int nnq, double fbts)
{
	CKalman::Feedback(nq, fbts);
#ifndef PSINS_FAST_CALCULATION
	sins.qnb -= *(CVect3*)&FBXk.dd[0];  sins.vn -= *(CVect3*)&FBXk.dd[3];
	sins.imu.eb  += *(CVect3*)&FBXk.dd[6];	sins.imu.db += *(CVect3*)&FBXk.dd[9];
	CMat3 IdKG = I33-(~(*(CMat3*)&FBXk.dd[12]));
	sins.imu.Kg = IdKG*sins.imu.Kg;			// dKG
	CMat3 IdKA(1.0-FBXk.dd[21],            0.0,           0.0,
			      -FBXk.dd[22],1.0-FBXk.dd[24],           0.0,
				  -FBXk.dd[23],   -FBXk.dd[25],1.0-FBXk.dd[26]);
	sins.imu.Ka = IdKA*sins.imu.Ka;			// dKA
	if(sins.imu.pKa2) sins.imu.Ka2 += *(CVect3*)&FBXk.dd[27];  // dKa2
	else			  sins.imu.Kapn += *(CVect3*)&FBXk.dd[27]; // dKapn
	sins.imu.lvx += *(CVect3*)&FBXk.dd[30];	sins.imu.lvy += *(CVect3*)&FBXk.dd[33];	// Inner lever
	sins.imu.tGA += FBXk.dd[36];	// tGA
#else
	qdelphi(sins.qnb, *(CVect3*)&FBXk.dd[0]);  VSUBE(sins.vn, (*(CVect3*)&FBXk.dd[3]));
	VADDE(sins.imu.eb, (*(CVect3*)&FBXk.dd[6]));  VADDE(sins.imu.db, (*(CVect3*)&FBXk.dd[9]));
	KgMdf(sins.imu.Kg, &FBXk.dd[12], 0);			// dKG
	KaMdf(sins.imu.Ka, &FBXk.dd[21], 0);			// dKA
	if(sins.imu.pKa2) { VADDE(sins.imu.Ka2, (*(CVect3*)&FBXk.dd[27])); } // dKa2
	else			  { VADDE(sins.imu.Kapn, (*(CVect3*)&FBXk.dd[27])); }// dKapn
	VADDE(sins.imu.lvx, (*(CVect3*)&FBXk.dd[30]));	VADDE(sins.imu.lvy, (*(CVect3*)&FBXk.dd[33]));	// Inner lever
	sins.imu.tGA += FBXk.dd[36];	// tGA
#endif
}

int CSysClbt::Update(const CVect3 *pwm, const CVect3 *pvm, int nn, double ts, int isStatic)
{
	if(isStatic==-1) isStatic=norm(sins.wib)<wibStatic;
	int nStep = isStatic ? 2 : 40;
	int res = TDUpdate(pwm, pvm, nn, ts, nStep);
	if(sins.mvnk==0) {
//		*(CVect3*)&Zk.dd[0] = sins.mvn;  sins.pos = pos0;
		*(CVect3*)&Zk.dd[0] = sins.vn;  sins.pos = pos0;
		if(isStatic) SetMeasFlag(07);
	}
	if(!isStatic) measflag=0;
	return res;
}

#ifdef PSINS_IO_FILE
void CSysClbt::Log(void)
{
	psinslog<<"Kg=["<<sins.imu.Kg<<"];\neb=["<<sins.imu.eb/glv.dph<<"]'; %dph\nKa=["<<sins.imu.Ka<<"];\ndb=["<<sins.imu.db/glv.ug<<"]'; %ug\n";
	psinslog<<"Ka2=["<<sins.imu.Ka2/glv.ugpg2<<"]'; %ugpg2\nlvx=["<<sins.imu.lvx*100<<"]'; %cm\nlvy=["<<sins.imu.lvy*100<<"]';\ntGA="<<sins.imu.tGA*1000<<"; %ms\n";
}

void CSysClbt::LogXk(void)
{
	psinslog.MArray("phi", &Xk.dd[0], 3, 1, 1/glv.min, "arcmin");
	psinslog.MArray("dVn", &Xk.dd[3], 3, 1, 1.0, "m/s");
	psinslog.MArray("eb", &Xk.dd[6], 3, 1, 1/glv.dph, "dph");
	psinslog.MArray("db", &Xk.dd[9], 3, 1, 1/glv.ug, "ug");
	CMat3 dKG = ~(*(CMat3*)&Xk.dd[12]);
	dKG.e00*=1/glv.ppm*glv.sec, dKG.e11*=1/glv.ppm*glv.sec, dKG.e22*=1/glv.ppm*glv.sec;
	psinslog.MArray("dKG", &dKG.e00, 3, 3, 1/glv.sec, "ppm|arcsec");
	CMat3 dKA(Xk.dd[21], 0.0,       0.0,
			  Xk.dd[22], Xk.dd[24], 0.0,
			  Xk.dd[23], Xk.dd[25], Xk.dd[26]);
	dKA.e00*=1/glv.ppm*glv.sec, dKA.e11*=1/glv.ppm*glv.sec, dKA.e22*=1/glv.ppm*glv.sec;
	psinslog.MArray("dKA", &dKA.e00, 3, 3, 1/glv.sec, "ppm|arcsec");
	if(sins.imu.pKa2)
		psinslog.MArray("dKa2", &Xk.dd[27], 3, 1, 1/glv.ugpg2, "ug/g^2");
	else
		psinslog.MArray("dKapn", &Xk.dd[27], 3, 1, 1/glv.ppm, "ppm");
	psinslog.MArray("Lvy", &Xk.dd[30], 3, 1, 100, "cm");
	psinslog.MArray("Lvz", &Xk.dd[33], 3, 1, 100, "cm");
	psinslog.MArray("tGA", &Xk.dd[36], 1, 1, 1000, "ms");
}
#endif

#ifdef PSINS_IO_FILE
#ifdef PSINS_RMEMORY
//***************************  class CPOS618  *********************************/
CPOS618::CPOS618(void)
{
	pmemImuGnss = pmemFusion = NULL;  psmth = NULL; fins = fkf = NULL;
}

CPOS618::CPOS618(double ts, double smthSec, BOOL isdebug):CSINSGNSS(19, 6, ts)
{
	this->ts = ts;  frq = (int)(1.0/ts+0.5);
	pmemImuGnss = pmemFusion = NULL;  fins = fkf = NULL;
	psmth = new CSmooth(9, (int)(smthSec/ts));
	if(isdebug) { fins = new CFileRdWt("insdebug.bin",0);  fkf = new CFileRdWt("kfdebug.bin",0); }
	lvGNSS = CVect3(0.5,7.5,2.5);
	dtGNSSdelay = 0.0;
}

CPOS618::~CPOS618()
{
	Deletep(pmemImuGnss); Deletep(pmemFusion); Deletep(pmemFusion1);
	Deletep(psmth);
	Deletep(fins); Deletep(fkf);
}

BOOL CPOS618::Load(char *fname, double t0, double t1)
{
	printf("--- Load data ---\n OK\n");
	CFileRdWt f(fname,-(int)(sizeof(ImuGnssData)/sizeof(double)));
	if(t0>EPS) f.load((int)(t0/ts));
	records = f.filesize()/sizeof(ImuGnssData);
	if(t1>EPS) {
		if(t1-1.0<t0) return FALSE;
		records = mmin(records,(int)((t1-t0)/ts));
	}
	pmemImuGnss = new CRMemory(records, sizeof(ImuGnssData));
	if(!pmemImuGnss) return FALSE;
	records = f.load(pmemImuGnss->get(0), records*sizeof(ImuGnssData)) / sizeof(ImuGnssData);
	for(iter=0,pIG=(ImuGnssData*)pmemImuGnss->get(0); iter<records; iter++,pIG++) {  // find first pos0
		if(!IsZero(pIG->posgnss.k)) { posgnss0 = pIG->posgnss; break; }
	}
	if(iter>=records) return FALSE;
	pmemFusion = new CRMemory(records, sizeof(FXPT));
	pmemFusion1 = new CRMemory(records, sizeof(FXPT));
	if(!(pmemFusion&&pmemFusion1)) return FALSE;
	return TRUE;
}

void CPOS618::Init(double talign, const CVect3 &att0)
{
	CVect3 att=att0;
	if(&att0==&O31)	{   // self-align
		CAligni0 aln(posgnss0);
		pIG=(ImuGnssData*)pmemImuGnss->get(0);
		for(double t=0.0; t<talign; t+=ts,pIG++)  aln.Update(&pIG->wm, &pIG->vm, 1, ts);
		att = q2att(aln.qnb0);
	}
	// KF Init
	pIG=(ImuGnssData*)pmemImuGnss->get(0);
	CSINSGNSS::Init(CSINS(att,O31,posgnss0,pIG->t-ts));
	Pmin.Set2(fPHI(0.1,1.0),  fXXX(0.0001),  fdPOS(0.0001),  fDPH3(0.001),  fUG3(10.0),	fXXX(0.001), 0.0001);
	Pk.SetDiag2(fPHI(60,600),  fXXX(0.1),  fdPOS(1.0),  fDPH3(0.1),  fMG3(1.0), fXXX(0.1),  0.0);  PmaxPminCheck();
	Qt.Set2(fDPSH3(0.01),  fXXZU(10,100,UGPSHZ),  fOOO,  fOO6,  fOOO, 0.0);
	Rt.Set2(fXXZ(0.1,0.3),   fdLLH(0.1,0.3));  Rt0 = Rt;
	Rmax = Rt*100;  Rmin = Rt*0.01;  Rb = 0.6f;
	FBTau = 10.0;
}

void CPOS618::Forward(void)
{
	printf("--- Forward processing ---\n");
	pIG=(ImuGnssData*)pmemImuGnss->get(0), pXP=(FXPT*)pmemFusion->get(0);
	for(iter=0; iter<records; iter++,pIG++,pXP++)
	{
		Update(&pIG->wm, &pIG->vm, 1, ts);
		if(pIG->posgnss.k>0.1||pIG->posgnss.k<-0.1)
			SetMeasGNSS(pIG->posgnss, pIG->vngnss);
		pXP->att = sins.att; pXP->vn = sins.vnL; pXP->pos = sins.posL;
		double *p=&pXP->Patt.i, *p1=&Pk.dd[0];
		for(int i=0; i<9; i++,p++,p1+=nq+1) *p = *p1;
		double *pp = iter>psmth->maxrow/2 ? &pXP->Patt.i-(psmth->maxrow/2*dbsize(FXPT)) : NULL;
		psmth->Update(&pXP->Patt.i, pp);  // Pk smooth
		pXP->t = sins.tk = this->kftk = pIG->t;
		if(iter%10==0&&fins) { *this<<*fins;  *fkf<<*this; }
		disp(iter,frq,100);
	}
}

void CPOS618::Backward(void)
{
	printf("--- Backward processing & Fusion ---\n");
//	pIG=(ImuGnssData*)pmemImuGnss->get(records), pXP1 = (FXPT*)pmemFusion1->get(records);
//	for(--iter,--pIG,--pXP,--pXP1; iter>=0; iter--,pIG--,pXP--,pXP1--)
	pIG=(ImuGnssData*)pmemImuGnss->get(-1), pXP=(FXPT*)pmemFusion->get(-1), pXP1=(FXPT*)pmemFusion1->get(-1);
	for(--iter,pIG,pXP,pXP1; iter>=0; iter--,pIG--,pXP--,pXP1--)
	{
		// fusion
		pXP1->att = sins.att; pXP1->vn = -sins.vnL; pXP1->pos = sins.posL;
		double *p=&pXP1->Patt.i, *p1=&Pk.dd[0];
		for(int i=0; i<9; i++,p++,p1+=nq+1) *p = *p1;
		pXP1->t = sins.tk = this->kftk = pIG->t;
		double *pxp1 = iter<records-psmth->maxrow/2 ? &pXP1->Patt.i+(psmth->maxrow/2*dbsize(FXPT)) : NULL;
		psmth->Update(&pXP1->Patt.i, pxp1);
		if(pxp1) {
			FXPT *px = pXP + psmth->maxrow/2, *px1 = pXP1 + psmth->maxrow/2;
			fusion(&px1->att.i, &px1->Patt.i, &px->att.i, &px->Patt.i, 9);
		}
		if(iter%10==0&&fins) { *this<<*fins;  *fkf<<*this; }
		// backward
		pIG->wm = -pIG->wm;  pIG->vngnss = -pIG->vngnss;
		if(pIG->posgnss.k>0.1||pIG->posgnss.k<-0.1)
			SetMeasGNSS(pIG->posgnss, pIG->vngnss);
		Update(&pIG->wm, &pIG->vm, 1, ts);
		disp(iter,frq,100);
	}
}

void CPOS618::Reverse(void)
{
	sins.eth.wie = -sins.eth.wie;  sins.vn = -sins.vn; sins.vnL = -sins.vnL; sins.eb = -sins.eb;  // reverse INS
	int idx[] = {3,4,5, 9,10,11, 18};  // reverse Xk
	for(int k=0; k<(sizeof(idx)/sizeof(int)); k++) Xk.dd[idx[k]] = -Xk.dd[idx[k]];
	TDReset();  avpi.Init(sins,kfts,1);  SetMeasStop(0xffffffff,2.0);
	int idxp[] = {0,1, 3,4,5, 6,7,8, 14};  // enlarge Pk
	Pset = diag(Pk);
	for(int p=0; p<(sizeof(idxp)/sizeof(int)); p++) Pset.dd[idxp[p]] = 10.0*Pset.dd[idxp[p]];
}

void CPOS618::Smooth(void)
{
	// see Matlab PSINS Toolbox \ POSSmooth.m
}

void CPOS618::Process(char *fname, int step)
{
	Forward();  if(step==1) return;
	Reverse();
	Backward();  if(step==2) return;
	Smooth();
	if(fname) {
		printf("--- Save results ---\n OK\n");
		CFileRdWt f(fname,0);
		f<<*pmemFusion1;   // struct FXPT 19-column record
	}
}
#endif  // PSINS_RMEMORY
#endif  // PSINS_IO_FILE

//***************************  class CSINSGNSSCNS  *********************************/
CSINSGNSSCNS::CSINSGNSSCNS(void)
{
}

CSINSGNSSCNS::CSINSGNSSCNS(double ts):CSINSGNSS(18, 9, ts)
{
	// 0-14: phi,dvn,dpos,eb,db;  15-17: mu^b_s
	Hk.SetMat3(6, 0, I33);  Hk(6,6)=1.0;	// SINS/CNS: Hk(7,7)=-sins.eth.cl; Hk(8,7)=-sins.eth.sl; Hk.SetMat3(6,15,sins.Cnb);
	Cbs = I33;
	SetMeasMask(0777);
}

void CSINSGNSSCNS::SetCNS(int year, int month, int day, double s0, double dUT1, double dTAI)
{
	cns.SetdT(dUT1, dTAI);
	cns.GetCie(cns.JD(year, month, day), s0);
}

void CSINSGNSSCNS::Init(const CSINS &sins0, int grade)
{
	CSINSGNSS::Init(sins0);
	Pmin.Set2(fPHI(0.01,0.01),  fXXX(0.001),  fdPOS(.1),  fDPH3(0.0001),  fUG3(1.0),  fXYZU(1,1,3,SEC));
	Pk.SetDiag2(fPHI(10,60),  fXXX(0.001),  fdPOS(10.0),  fDPH3(0.01),  fUG3(100.0),  fXYZU(10,10,30,MIN));
	Qt.Set2(fDPSH3(0.001),  fUGPSHZ3(1.0),  fOO9,  fOOO);
	Rt.Set2(fXXZ(0.5,1.0),   fdLLH(10.0,30.0), fXYZU(10,10,30,SEC));  Rt0 = Rt;
	Rmax = Rt*100;  Rmin = Rt*0.01;  //Rb = 0.6;
	FBTau.Set(fXYZ(0.1,0.1,0.1),  fXX6(0.1),  fXX6(1.0),  fXXX(INF));
}

void CSINSGNSSCNS::SetHk(int nnq)
{
	Hk(7,7)=-sins.eth.cl; Hk(8,7)=-sins.eth.sl; Hk.SetMat3(6,15,sins.Cnb);
}

void CSINSGNSSCNS::Feedback(int nnq, double fbts)
{
	CSINSGNSS::Feedback(15, fbts);
	Cbs = Cbs*a2mat(*(CVect3*)&FBXk.dd[15]);
}

void CSINSGNSSCNS::SetMeasCNS(CQuat &qis)
{
	if(qis.q0<=0) return;
	CMat3 mtmp = cns.GetCns(qis, sins.pos, sins.tk, Cbs);
	*(CVect3*)&Zk.dd[6] = sins.qnb - m2qua(mtmp);
	SetMeasFlag(00700);
}

void CSINSGNSSCNS::SetMeasCNS(CVect3 &vqis)
{
	double q0=1.0-dot(vqis,vqis);
	q0 = q0>0 ? sqrt(q0) : 0.0;
	CQuat qtmp = CQuat(q0, vqis);
	SetMeasCNS(qtmp);
}

//***************************  class CSINSGNSSDR  *********************************/
CSINSGNSSDR::CSINSGNSSDR(void)
{
}

CSINSGNSSDR::CSINSGNSSDR(double ts):CSINSGNSS(21, 13, ts)
{
	// 0-14: phi,dvn,dpos,eb,db; 15-17: Kappa; 18-20: dposDR
	Kod = 1.0;
	lvOD = O31;  Cbo = I33;
	Hk(6,2) = 0.0;  Hk.SetMat3(6, 6, I33);   Hk.SetMat3(6, 18, -I33*0);  // SINS/DR-dpos
	Hk.SetMat3(9, 18, -I33);  // GNSS/DR-dpos
	Hk(12,2) = 1.0;  // SINS/GNSS-dyaw
}

void CSINSGNSSDR::Init(const CSINS &sins0, int grade)
{
	CSINSGNSS::Init(sins0, grade);  // sins0.pos is GNSS pos
	sins.lever(-lvGNSS);	sins.pos = sins.posL;
	sins.lever(lvOD);		posDR = sins.posL,  sins.posL = posDRL = sins0.pos;
	avpi.Init(sins, kfts);
	Pmax.Set2(fDEG3(10.0), fXXX(50.0), fdPOS(1.0e4),
		fDPH3(3600), fMG3(10.0), 1*DEG, 0.01, 1*DEG, fdPOS(1.0e4));
	Pmin.Set2(fPHI(10.0,60.0), fXXX(0.01), fdPOS(0.1),
		fDPH3(0.1), fUG3(100), 0.01*DEG, 0.001, 0.01*DEG, fdPOS(0.1));
	Pk.SetDiag2(fDEG3(10.0), fXXX(1.0), fdPOS(100.0),
		fDPH3(1000), fMG3(1.0), 1*DEG, 0.01, 1*DEG, fdPOS(100.0));
	Qt.Set2(fDPSH3(1), fUGPSHZ3(10), fOO9, fOO6);
	Rt.Set2(fXXZ(0.2,0.6), fdLLH(10.0,30.0), fdLLH(1.1,1.0), fdLLH(10,10), 1.0*DEG);  Rt0 = Rt;
	Rmax = Rt*100;  Rmin = Rt*0.01;  Rb = 0.5;
	FBTau.Set(fIII, fIII, fIII, fIII, fIII, fIII, fIII);
}

void CSINSGNSSDR::SetFt(int nnq)
{
	CSINSGNSS::SetFt(15);
	CMat3 MvkD = norm(sins.vn)*CMat3(-sins.Cnb.e02,sins.Cnb.e01,sins.Cnb.e00,
			-sins.Cnb.e12,sins.Cnb.e11,sins.Cnb.e10,-sins.Cnb.e22,sins.Cnb.e21,sins.Cnb.e20);
	Ft.SetMat3(18, 0, sins.Mpv*askew(sins.vn));
	Ft.SetMat3(18, 15, sins.Mpv*MvkD, sins.Mpp);
}

void CSINSGNSSDR::Feedback(int nnq, double fbts)
{
	CSINSGNSS::Feedback(15, fbts);
	Cbo = Cbo*a2mat(CVect3(FBXk.dd[15],0.0,FBXk.dd[17]));
	Kod *= 1 - FBXk.dd[16];
	posDR -= *(CVect3*)(&FBXk.dd[18]);
}

void CSINSGNSSDR::SetMeas(void)
{
	if(*gnssLost>5.0)
	{
		*(CVect3*)&Zk.dd[6] = sins.posL - posDRL;  // SINS/DR
		SetMeasFlag(000700);
	}
}

void CSINSGNSSDR::SetMeasGNSS(const CVect3 &pgnss, const CVect3 &vgnss, double yawgnss)
{
	if(!IsZero(pgnss) && avpi.Interp(posGNSSdelay))
	{
		*(CVect3*)&Zk.dd[3] = avpi.pos - pgnss;  // SINS/GNSS
		SetMeasFlag(000070);
		*(CVect3*)&Zk.dd[9] = pgnss - posDRL;	// GNSS/DR
		SetMeasFlag(007000);
	}
	if(!IsZero(vgnss) && avpi.Interp(vnGNSSdelay))
	{
		*(CVect3*)&Zk.dd[0] = avpi.vn - vgnss;
		SetMeasFlag(000007);
	}
	if(!IsZero(yawgnss) && 
		IsZero(sins.att.i,10*DEG) && IsZero(sins.att.j,10*DEG) && IsZero(sins.wnb.k,10*DPS))
	{
		Zk.dd[12] = -diffYaw(sins.att.k, yawgnss+dyawGNSS);
		SetMeasFlag(010000);
	}
}

int CSINSGNSSDR::Update(const CVect3 *pwm, const CVect3 *pvm, double dS, int nn, double ts, int nSteps)
{
	int res = CSINSGNSS::Update(pwm, pvm, nn, ts, nSteps); 
	CVect3 dSn = sins.Cnb*(Cbo*CVect3(0,dS*Kod,0));
	posDR = posDR + sins.eth.vn2dpos(dSn, 1.0);  // DR update
	posDRL = posDR + sins.MpvCnb*(lvGNSS-lvOD);  // trans posDRL to GNSS
	return res;
}

//***************************  class CSINSGNSSOD  *********************************/
CSINSGNSSOD::CSINSGNSSOD(void)
{
}

CSINSGNSSOD::CSINSGNSSOD(int nq0, int nr0, double ts, int yawHkRow0):CSINSGNSS(nq0, nr0, ts, yawHkRow0)
{
	odmeast = 0.0;  odmeasT = 1.0;  dS0 = 0.0;  Hkv2y = 1.0;  SetDistance(0.0);
	lvOD = vnOD = O31;  Kod2 = AOS = 0.0;  ODKappa(CVect3(0,1,0));
	odmeasOK = badODCnt = IVniFirst = 0;  afaIVni = 1.0;
}

void CSINSGNSSOD::Init(const CSINS &sins0, int grade)
{
	CSINSGNSS::Init(sins0, grade);
	sins.lever(lvOD, &posOD, &vnOD);
	odmeasT = 1.0;
}

void CSINSGNSSOD::SetDistance(double dist)
{
	distance = dist;
	for(int k=0; k<distN; k++) { distances[k]=distance; }
	distK0=0; distT01=0.0; odVel=0.0;
}

CVect3 CSINSGNSSOD::ODKappa(const CVect3 &kpp)
{
	CVect3 res;
	if(&kpp==&O31) {		// get
		res = m2att(Cbo);  res.j = Kod;  // Kappa = [dPitch; Kod; dYaw]
	}
	else {					// set
		Kod = kpp.j;
		Cbo = a2mat(CVect3(kpp.i,0.0,kpp.k));
		res = O31;
	}
	return res;
}

int CSINSGNSSOD::ODVelUpdate(double dS)
{
	if(dS<-1.0||dS>10.0) { dS = dS0; badODCnt++; } else { badODCnt=0; }; dS0 = dS;  // if bad OD input 
	if(odmeast<EPS || badODCnt>5) {   // re-initialize
		IVno = IVni = Ifn = O31;
		IMv = odMlever = O33;
		if(badODCnt>5) IVniFirst=0;
	}
	dS *= Kod;
	distance += dS;
	if(++distK0==distN) { distK0=0; } distT01=distance-distances[distK0]; distances[distK0]=distance;  // distT01=dist increment within [T0,T1]
	odVel = distT01/(distN*sins.nts);
	CVect3 dSn = sins.Cnb*(CVect3(Cbo.e01*dS,Cbo.e11*dS,Cbo.e21*dS)-sins.imu.phim*lvOD);  // dSn = Cnb*Cbo*dSb
	odmeast += sins.nts;
	if(odmeast>=odmeasT) {  // take measurement every (odmeasT)(s)
		odMphi = -askew(odmeast/2.0*Ifn+IVno);  odMphi.e02*=Hkv2y,odMphi.e12*=Hkv2y;
		odMvn = odmeast*I33;
		odMkappa = CMat3( IMv.e02,-IMv.e01,-IMv.e00,
						  IMv.e12,-IMv.e11,-IMv.e10,
						  IMv.e22,-IMv.e21,-IMv.e20 );
		//odMlever = sins.Cnb*askew(-odmeast*sins.web);
		if(Kod2>EPS) { double k2=Kod2/odmeast; IVno.i += k2*IVno.i*IVno.i;  IVno.j += k2*IVno.j*IVno.j; } // 2nd nonlinear scale factor
		if(AOS>EPS) { IVno = rotz(IVno, -AOS*sins.web.k); } // Angle Of Slide correct
		odZk = IVni - IVno; 
		odmeast = 0.0;
//		return odmeasOK=1;
		return odmeasOK = badODCnt<5 ? 1 : 0;
	}
	else {
		IVno += dSn;  IVni += sins.vn*sins.nts; Ifn += sins.fn*sins.nts;
		IMv += dS*sins.Cnb;
		odMlever += sins.Cnb*askew(-sins.nts*sins.web);
		return odmeasOK=0;
	}
}

int CSINSGNSSOD::Update(const CVect3 *pwm, const CVect3 *pvm, double dS, int nn, double ts, int nSteps)
{
	int res = CSINSGNSS::Update(pwm, pvm, nn, ts, nSteps);
	sins.lever(lvOD, &posOD, &vnOD);
	ODVelUpdate(dS);
	return res;
}

#ifdef PSINS_IO_FILE
void CSINSGNSSOD::operator<<(CFileRdWt &f)
{
	f<<sins.att<<sins.vn<<sins.pos<<sins.eb<<sins.db  // 1-15
		<<sins.vnL<<sins.posL<<lvGNSS   // 16-24
		<<vnOD<<posOD<<lvOD<<ODKappa()  // 25-36
		<<(navStatus+dtGNSSdelay)<<dyawGNSS<<(sins.Kg.e22-1.0) <<kftk; // 37-40
}
#endif

//***************************  class CAutoDrive  *********************************/
CAutoDrive::CAutoDrive(void)
{
}

CAutoDrive::CAutoDrive(double ts):CSINSGNSSOD(18, 17, ts, 15)
{
	// 0-14: phi,dvn,dpos,eb,db; 15-17: Kappa;
	gnssLostdist = gnssLostnofixdist = nofixYaw0 = 0.0;
	odLost = &measlost.dd[6];  zuptLost = &measlost.dd[9];
	// Hk(0:5,:) ...		// 0-5: SINS/GNSS-dvn,dpos
	Hk.SetMat3(6, 3, I33);  // 6-8: SINS/OD-dvn
	Hk.SetMat3(9, 3, I33);  // 9-11: ZUPT
	Hk.SetMat3(12, 3, I33); // 12,14: NHC
	Hk(15,2) = 1.0;			// 15: SINS/GNSS-dyaw
	Hk(16,11) = 1.0;		// 16: WzHold (ZIHR)
	SetMeasMask(0357777);
	pPkPhiu = &Pk(2,2);  pPkVu = &Pk(5,5);  pPkHgt = &Pk(8,8);
}

void CAutoDrive::Init(const CSINS &sins0, int grade)
{
	CSINSGNSSOD::Init(sins0, grade);
	wzhd.Init(200.0*DPH, kfts, 10.0);
	Pmax.Set2(fDEG3(10.0), fXXX(50.0), fdPOS(1.0e4), fDPH3(3600), fMG3(10.0), fKPP(1,0.01,1));
	Pmin.Set2(fPHI(10.0,60.0), fXXX(0.01), fdPOS(0.1), fDPH3(0.1), fUG3(100), fKPP(0.01,0.001,0.01));
	Pk.SetDiag2(fDEG3(10.0), fXXX(1.0), fdPOS(100.0), fDPH3(1000), fMG3(1.0), fKPP(1,0.01,1));
	Qt.Set2(fDPSH3(1), fUGPSHZ3(10), fOO9, fOOO);
	Rt.Set2(fXXZ(0.2,0.6), fdLLH(10.0,30.0), 
		fXXZ(0.1,0.1), fXXZ(0.1,0.1), fXYZ(0.1,10.0,0.1), 1.0*DEG, 10.0*DPH);  Rt0 = Rt;
	Rmax = Rt*100;  Rmin = Rt*0.01;  Rb.SetBit(0100077, 0.5);
	*(CVect3*)&Zmax.dd[0] = CVect3(2.0,2.0,0.5);
	FBTau = 1.0;
	SetGNSSFixMode(0);
}

void CAutoDrive::SetFt(int nnq)
{
	CSINSGNSS::SetFt(15);
}

void CAutoDrive::SetHk(int nnq)
{
}

void CAutoDrive::Feedback(int nnq, double fbts)
{
	CSINSGNSS::Feedback(15, fbts);
	Cbo = Cbo*a2mat(CVect3(FBXk.dd[15],0.0,FBXk.dd[17]));
	Kod *= 1 - FBXk.dd[16];
}

void CAutoDrive::ZUPTtest(void)
{
	if(sins.mmwb.flag==1 && odVel<1.0e-6) {
		double dwb = sins.mmwb.maxRes-sins.mmwb.minRes,
			dfb = sins.mmfb.maxRes-sins.mmfb.minRes;
		if(dwb<0.1*DPS && sins.mmwb.maxRes<1.0*DPS && dfb<50*MG && sins.mmfb.maxRes<50*MG) {
			*(CVect3*)&Zk.dd[9] = sins.vn;
			SetMeasFlag(0007000);
		}
	}
}

void CAutoDrive::ZIHRtest(void)
{
	wzhd.Update(sins.imu.phim.k/sins.nts);
	if(wzhd.retres==3 && sins.fn.k>9.5 && odVel<1.0e-6) {
//		Zk.dd[16] = wzhd.meanwpre-sins.eth.wnie.k;
		Zk.dd[16] = wzhd.meanw-sins.eth.wnie.k;
		SetMeasFlag(0200000);     SetMeasStop(0100000,wzhd.T);
		Xk.dd[2] -= Zk.dd[16]*wzhd.T;    //	sins.qnb -= CVect3(0,0,-Zk.dd[16]*wzhd.T);
	}
}

void CAutoDrive::NHCtest(void)
{
	if(IsZero(sins.wnb.k,20*DPS)) {
		CMat3 Con = (~Cbo)*sins.Cbn;  CVect3 vo = Con*sins.vn;
		Hk.SetMat3(12,0,Con*askew(-sins.vn),Con);  Hk.SetAskew(12,15,vo);  // FuQW,2012
		*(CVect3*)&Zk.dd[12] = vo;
		SetMeasFlag(050000);
	}
}

void CAutoDrive::SetGNSSFixMode(int mode)
{
	if(mode==0)	{ // init or non-GNSS
		fixLost = fixLast = nofixLost = nofixLast = gnssLast = 0;
	}
	else if(mode==1) { // fixed mode
		fixLost=0;  fixLast++;  nofixLost++;  nofixLast=0;  gnssLast++;
		if(fixLast>3) { gnssLostdist = 0.0; gnssLostnofixdist = 0.0; }
	}
	else if(mode==2) { // non-fixed mode
		fixLost++;  fixLast=0;  nofixLost=0;  nofixLast++;  gnssLast++;
		if(nofixLast>5) { gnssLostdist = 0.0; }
		if(nofixLast==1) { nofixYaw0 = sins.att.k; }
	}
}

int CAutoDrive::Update(const CVect3 *pwm, const CVect3 *pvm, double dS, int nn, double ts, int nSteps)
{
	int res = CSINSGNSSOD::Update(pwm, pvm, dS, nn, ts, nSteps);
	if(odmeasOK) {
		Hk.SetMat3(6, 0, odMphi, odMvn); Hk.SetMat3(6, 15, odMkappa);
//		*(CVect3*)&Zk.dd[6] = odZk;  // SINS/OD-dvn
//		SetMeasFlag(0700);
		*(CVect3*)&Zk.dd[6] = IVni_1*(1.0-afaIVni)+IVni*afaIVni-IVno;  // SINS/OD-dvn
		IVni_1 = IVni;
		if(IVniFirst++) SetMeasFlag(0700);
	}
	gnssLostdist += dS;  gnssLostnofixdist += dS;
	if(*gnssLost>10 && *odLost>10.0) ZUPTtest();
	ZIHRtest();
	if(*gnssLost>10 && *odLost>10 && *zuptLost>10) NHCtest();
	return res;
}

//***************************  class CSGOClbt  *********************************/
CSGOClbt::CSGOClbt(double ts):CSINSGNSSOD(27, 10, ts, 9)
{
	// 0-14: phi,dvn,dpos,eb,db; 15-17: lvGNSS; 18-20: Kappa; 21-23: lvOD; 24: dtGNSS; 25: dyawGNSS; 26: dKGzz
	// Hk(0:5,:) ...								// 0-5: SINS/GNSS-dvn,dpos
	Hk.SetMat3(6, 3, I33);							// 6-8: SINS/OD-dvn
	Hk(yawHkRow,2) = 1.0; Hk(yawHkRow,25) = -1.0; 	// 9: SINS/GNSS-dyaw
	SetMeasMask(01777);
}

void CSGOClbt::Init(const CSINS &sins0, int grade)
{
	CSINSGNSSOD::Init(sins0, grade);
	if(grade<0) return;
	Pmax.Set2(fDEG3(10.0), fXXX(50.0), fdPOS(1.0e4),
		fDPH3(3600), fMG3(10.0), fXXX(1.1), fKPP(1,0.01,1), fXXX(10.0), 0.1, 1*DEG, 1000*PPM);
	Pmin.Set2(fPHI(0.1,0.6), fXXX(0.001), fdPOS(0.01),
		fDPH3(0.1), fUG3(100), fXXZ(0.01,0.01), fKPP(0.01,0.001,0.01), fXXX(0.001), 0.0001, 0.01*DEG, 0.0);
	Pk.SetDiag2(fDEG3(1.0), fXXX(1.0), fdPOS(100.0),
		fDPH3(10), fMG3(1.0), fXXZ(1.1,1.1), fKPP(0.1,0.01,1.1), fXXX(1.0), 0.01, 1.1*DEG, 1000*PPM);
	Qt.Set2(fDPSH3(0.51), fUGPSHZ3(1000), fOO9, fOOO, fOOO, fOOO, 0.0, 0.0, 0.0);
	Rt.Set2(fXXZ(0.2,0.6), fdLLH(10.0,30.0), fXXZ(0.1,0.1), 1.0*DEG);  Rt0 = Rt;
	Rmax = Rt*100;  Rmin = Rt*0.0001;  Rb.SetBit(077, 0.5);
	FBTau = .10;
}

void CSGOClbt::SetFt(int nnq)
{
	CSINSGNSS::SetFt(18);
	Ft(2,26) = -sins.wib.k*sins.Cnb.e22;  // 26 dKGzz
}

void CSGOClbt::SetHk(int nnq)
{
	CSINSGNSS::SetHk(18);
	CVect3 MV=sins.Mpv*sins.vn;
	Hk.SetClmVect3(0,24, -sins.an);
	Hk.SetClmVect3(3,24, -MV);
}

void CSGOClbt::Feedback(int nnq, double fbts)
{
	CSINSGNSS::Feedback(18, fbts);
	Cbo = Cbo*a2mat(CVect3(FBXk.dd[18],0.0,FBXk.dd[20]));
	Kod *= 1.0-FBXk.dd[19];
	lvOD += *(CVect3*)&FBXk.dd[21]; 
	dtGNSSdelay += FBXk.dd[24];
	dyawGNSS += FBXk.dd[25];
	sins.Kg.e22 *= 1.0-FBXk.dd[26];
}

int CSGOClbt::Update(const CVect3 *pwm, const CVect3 *pvm, double dS, int nn, double ts, int nSteps)
{
	int res = CSINSGNSSOD::Update(pwm, pvm, dS, nn, ts, nSteps);
	if(odmeasOK) {
		Hk.SetMat3(6, 0, odMphi, odMvn); Hk.SetMat3(6, 18, odMkappa, odMlever);  // lvOD
		*(CVect3*)&Zk.dd[6] = odZk;  // SINS/OD-dvn
		SetMeasFlag(0700);
	}
	return res;
}

//***************************  class CVAutoPOS  *********************************/
CVAutoPOS::CVAutoPOS(void)
{
}

CVAutoPOS::CVAutoPOS(double ts):CSINSGNSSOD(18,3,ts)    //(18,3)
{
}

void CVAutoPOS::Init(const CSINS &sins0, int grade)
{
	CSINSGNSSOD::Init(sins0,grade);
	Pmin.Set2(fPHI(0.01,.1), fXXX(0.0001), fdPOS(0.001),
		fDPH3(0.001), fXYZU(10,10,50,UG), 0.0*DEG, 0.000, 0.0*DEG);
	// States   0-14: phi,dvn,dpos,eb,db; 15-17: Kappa;
	Pk.SetDiag2(fPHI(3.0,10.0), fXXX(0.5), fdPOS(1.0),
		fDPH3(0.05), fXYZU(50,50,1000,UG), 0.5*DEG, 0.01, 0.50*DEG);
	Qt.Set2(fDPSH3(0.005), fUGPSHZ3(10), fOO9, fOOO);
	// Meas     0-3: SINS/OD-dvn
	Rt.Set2(fXYZ(0.1,0.1,0.1));  Rt0 = Rt;
	FBTau = 1.0;
}

void CVAutoPOS::SetFt(int nnq)
{
	CSINSGNSS::SetFt(15);
}

void CVAutoPOS::SetHk(int nnq)
{
}

void CVAutoPOS::Feedback(int nnq, double fbts)
{
	CSINSGNSS::Feedback(15, fbts);
	Cbo = Cbo*a2mat(CVect3(FBXk.dd[15],0.0,FBXk.dd[17]));
	Kod *= 1 - FBXk.dd[16];
}

int CVAutoPOS::Update(const CVect3 *pwm, const CVect3 *pvm, double dS, int nn, double ts, int nSteps)
{
	int res = CSINSGNSSOD::Update(pwm, pvm, dS, nn, ts, nSteps);
	if(odmeasOK) {
		Hk.SetMat3(0, 0, odMphi, odMvn); Hk.SetMat3(0, 15, odMkappa);
		*(CVect3*)&Zk.dd[0] = odZk;  // SINS/OD-dvn
		SetMeasFlag(07);
	}
	return 1;
}

//***************************  class CCAM & CCALLH  *********************************/
void CCAM::Init(const CVect3 &pva, const CVect3 &qt, double rp, double rv, const CVect3 &pva0)
{
	Phi = I33;  Xk = O31;
	Pk = diag(pva);  Pmin = pva0;
	Qt = pow(qt,2);
	Rpk = pow2(rp); Rvk=pow2(rv);
}

void CCAM::TUpdate(double ts, double an)
{
	Phi.e01 = Phi.e12 = ts;		// Phi=[1 ts 0; 0 1 ts; 0 0 1]
	Xk = Phi*Xk;  Xk.j+=an*ts;	// Xkk_1=Phi*Xk+[0;a*nts;0]
	Pk = Phi*Pk*(~Phi)+Qt*ts;	// Pkk_1=Phi*Pk*(~Phi)+Qk
}

void CCAM::MUpdate(double Zpk, double Zvk)
{
	CVect3 PHT, Kk;
	if(!IsZero(Zpk)) {
		PHT = Pk.GetClm(0);				// PHT=Pkk_1*Hpk, where Hpk=[1,0,0]
		Kk = PHT*(1.0/(PHT.i+Rpk));		// PHT.i=dot(Hpk,PHT)
		Xk = Xk+Kk*(Zpk-Xk.i);			// Xkk_1.i=dot(Hpk,Xkk_1)
		Pk = (I33-CMat3(Kk,O31,O31,0))*Pk;  // vxv(Kk,Hpk)=CMat3(Kk,O31,O31,0)
	}
	if(!IsZero(Zvk)) {
		PHT = Pk.GetClm(1);				// PHT=Pkk_1*Hvk, where Hvk=[0,1,0]
		Kk = PHT*(1.0/(PHT.j+Rvk));
		Xk = Xk+Kk*(Zvk-Xk.j);
		Pk = (I33-CMat3(O31,Kk,O31,0))*Pk;
	}
	if(Pk.e00<Pmin.i) Pk.e00=Pmin.i;
	if(Pk.e11<Pmin.j) Pk.e11=Pmin.j;
	if(Pk.e22<Pmin.k) Pk.e22=Pmin.k;
	symmetry(Pk);
}

void CCAM::Update(double ts, double an, double Zpk, double Zvk)
{
	TUpdate(ts, an);
	MUpdate(Zpk, Zvk);
}

void CCALLH::Init(const CVect3 &pva, const CVect3 &qt, double rp, double rv, const CVect3 &pva0)
{
	lat.Init(pva, qt, rp, rv, pva0);
	lon.Init(pva, qt, rp, rv, pva0);
	hgt.Init(pva, qt, rp, rv, pva0);
	tk = 0.0;
}

void CCALLH::Update(double ts)
{
	tk += ts;
	lat.TUpdate(ts);  lon.TUpdate(ts);  hgt.TUpdate(ts);
}

void CCALLH::Update(const CVect3 &vn)
{
	lat.MUpdate(0.0, vn.j);  lon.MUpdate(0.0, vn.i);  hgt.MUpdate(0.0, vn.k);
}

void CCALLH::Update(double ts, const CVect3 &vn)
{
	Update(ts);
	Update(vn);
}

CVect3 CCALLH::GetdPos(const CSINS &sins)
{
	return CVect3(lat.Xk.i/sins.eth.RMh, lon.Xk.i/sins.eth.clRNh, hgt.Xk.i);
}

CVect3 CCALLH::GetdVn(void)
{
	return CVect3(lon.Xk.j, lat.Xk.j, hgt.Xk.j);
}

CVect3 CCALLH::GetPhi(double *dbU)
{
	if(dbU) *dbU=hgt.Xk.k;
	return CVect3(lat.Xk.k/G0, -lon.Xk.k/G0, 0.0);
}

double CCALLH::GetAVP(CVect3 &att, CVect3 &vn, CVect3 &pos, const CSINS &sins)
{
	double dbU;
	att=q2att(sins.qnb-GetPhi(&dbU)); vn=sins.vn-GetdVn(); pos=sins.pos-GetdPos(sins);
	return dbU;
}

/*
void CCALLH::Init(CSINS &sins, const CVect3 &lv) {
	lat.Init(CVect3(sins.pos.i,sins.vn.j,0), CVect3(1.0/RE/SHUR,1.0/SHUR,0.510*G0/SHUR), 0.1/RE, 0.01);
	lon.Init(CVect3(sins.pos.j,sins.vn.i,0), CVect3(1.0/RE/SHUR,1.0/SHUR,0.510*G0/SHUR), 0.1/RE, 0.01);
	hgt.Init(CVect3(sins.pos.k,sins.vn.k,0), CVect3(1.0/SHUR,   1.0/SHUR,0.510*G0/SHUR), 0.1,    0.01);
	lever = lv;
	OutLLH();  tk = sins.tk;
}
	
void CCALLH::Update(const CSINS &sins, const CVect3 &posGPS, const CVect3 &vnGPS)
{
	CVect3 an = sins.an + sins.Cnb*(sins.wib*(sins.wib*lever));
	lat.Phi.e01 = sins.eth.f_RMh*sins.nts;  lat.Phi.e12 = sins.nts;
	lat.Update(an.j, sins.nts, posGPS.i, vnGPS.j);
	lon.Phi.e01 = sins.eth.f_clRNh*sins.nts; lon.Phi.e12 = sins.nts;
	lon.Update(an.i, sins.nts, posGPS.j, vnGPS.i);
	hgt.Phi.e01 = sins.nts; hgt.Phi.e12 = sins.nts;
	hgt.Update(an.k, sins.nts, posGPS.k, vnGPS.k);
	OutLLH();  tk = sins.tk;
}

void CCALLH::Update(const CVect3 &posGPS, const CVect3 &vnGPS)
{
	lat.Update(posGPS.i, vnGPS.j);
	lon.Update(posGPS.j, vnGPS.i);
	hgt.Update(posGPS.k, vnGPS.k);
	OutLLH();
}
	
void CCALLH::OutLLH(void)
{
	db  = CVect3(lon.Xk.k, lat.Xk.k, hgt.Xk.k);	Pdb  = CVect3(lon.Pk.e22, lat.Pk.e22, hgt.Pk.e22);
	vn  = CVect3(lon.Xk.j, lat.Xk.j, hgt.Xk.j);	Pvn  = CVect3(lon.Pk.e11, lat.Pk.e11, hgt.Pk.e11);
	pos = CVect3(lat.Xk.i, lon.Xk.i, hgt.Xk.i);	Ppos = CVect3(lat.Pk.e00, lon.Pk.e00, hgt.Pk.e00);
}
*/
//***************************  class CGKP  *********************************/
CGKP::CGKP(double a0, double f0)
{
	Init(a0, f0);
}

void CGKP::Init(double a0, double f0)
{
    int k;
    double n, nk[6], e, ep;

    a = a0;
    b = (1-f0)*a;
    e = sqrt(a*a-b*b)/a;    e2 = e*e;
    ep = sqrt(a*a-b*b)/b;    ep2 = ep*ep;

    for(k=1,nk[0]=1,n=(a-b)/(a+b); k<6; k++)
        nk[k] = nk[k-1]*n;

    cp[1] = (a+b)/2*(1+1./4*nk[2]+1./64*nk[4]);
    cp[2] = -3./2*nk[1]+9./16*nk[3]-3./32*nk[5];
    cp[3] = 15./16*nk[2]-15./32*nk[4];
    cp[4] = -35./48*nk[3]+105./256*nk[5];
    cp[5] = 315./512*nk[4];
    cn[1] = (a+b)/2*(1+1./4*nk[2]+1./64*nk[4]);
    cn[2] = 3./2*nk[1]-27./32*nk[3]+269./512*nk[5];
    cn[3] = 21./16*nk[2]-55./32*nk[4];
    cn[4] = 151./96*nk[3]-417./128*nk[5];
    cn[5] = 1097./512*nk[4];
}

CVect3 CGKP::GKPCore(const CVect3 &BL)
{
    int k;
    double B, l, x, y;
    double lB, t, tk[7], lk[9], sB, cB, cBk[9], eta2, eta4, N;

    B = BL.i, l = BL.j;
    lB = B;
    for(k=1; k<5; k++)
        lB += cp[k+1]*sin(2*k*B);
    lB *= cp[1];
    
    cB = cos(B);
    cBk[0] = 1;
    for(k=1; k<9; k++)
        cBk[k] = cBk[k-1]*cB;
    t = tan(B);
    tk[0] = 1;
    for(k=1; k<7; k++)
        tk[k] = tk[k-1]*t;
    lk[0] = 1;
    for(k=1; k<9; k++)
        lk[k] = lk[k-1]*l;
    eta2 = ep2*cBk[2], eta4 = eta2*eta2;
    sB = sin(B);
    N = a/sqrt(1-e2*sB*sB);
    
    x = lB + t/2*N*cBk[2]*lk[2] + t/24*N*cBk[4]*(5-tk[2]+9*eta2+4*eta4)*lk[4] \
        + t/720*N*cBk[6]*(61+58*tk[2]+tk[4]+270*eta2-330*tk[2]*eta2)*lk[6] \
        + t/40320*N*cBk[8]*(1385-3111*tk[2]+543*tk[4]-tk[6])*lk[8];
    y = N*cB*l + 1./6*N*cBk[3]*(1-tk[2]+eta2)*lk[3] \
        + 1./120*N*cBk[5]*(5+18*tk[2]+tk[4]+14*eta2-58*tk[2]*eta2)*lk[5] \
        + 1./5040*N*cBk[7]*(61-479*tk[2]+179*tk[4]-tk[6])*lk[7];
    return CVect3(x, y, BL.k);
}

CVect3 CGKP::IGKPCore(const CVect3 &xy)
{
    int k;
    double B, l, x, y;
    double xbar, Bf, Nf, Nfk[9], tf, tfk[7], yk[9], sBf, cBf, etaf2, etaf4;

    x = xy.i, y = xy.j;
    xbar = x/cp[1];
    Bf = xbar;
    for(k=1; k<5; k++)
        Bf += cn[k+1]*sin(2*k*xbar);
    
    sBf = sin(Bf);
    Nf = a/sqrt(1-e2*sBf*sBf);
    Nfk[0] = 1;
    for(k=1; k<9; k++)
        Nfk[k] = Nfk[k-1]*Nf;
    tf = tan(Bf);
    tfk[0] = 1;
    for(k=1; k<7; k++)
        tfk[k] = tfk[k-1]*tf;
    cBf = cos(Bf);
    etaf2 = ep2*cBf*cBf, etaf4 = etaf2*etaf2;
    yk[0] = 1;
    for(k=1; k<9; k++)
        yk[k] = yk[k-1]*y;
    
    B = Bf + tf/(2*Nfk[2])*(-1-etaf2)*yk[2] \
        + tf/(24*Nfk[4])*(5-3*tfk[2]+6*etaf2-6*tfk[2]*etaf2-3*etaf4-9*tfk[2]*etaf4)*yk[4] \
        + tf/(720*Nfk[6])*(-61-90*tfk[2]-45*tfk[4]-107*etaf2+162*tfk[2]*etaf2+45*tfk[4]*etaf2)*yk[6] \
        + tf/(40320*Nfk[8])*(1385+3633*tfk[2]+4095*tfk[4]+1575*tfk[6])*yk[8];
    l = 1./(Nf*cBf)*y + 1./(6*Nfk[3]*cBf)*(-1-2*tfk[2]-etaf2)*yk[3] \
        + 1./(120*Nfk[5]*cBf)*(5-28*tfk[2]+24*tfk[4]*6*etaf2+8*tfk[2]*etaf2)*yk[5] \
        + 1./(5040*Nfk[7]*cBf)*(-61-662*tfk[2]-1320*tfk[4]-720*tfk[6])*yk[7];
    return CVect3(B, l, xy.k);
}

CVect3 CGKP::GKP(const CVect3 &BL)
{
    int stripNo = (int)((BL.j/DEG)/6+1);
    CVect3 BLH(BL.i, BL.j-(stripNo*6-3)*DEG, BL.k), xy=GKPCore(BLH);
    xy.j += stripNo*1000000+500000;  // x/i to North; y/j to East
    return xy;
}

CVect3 CGKP::IGKP(const CVect3 &xy)
{
    int stripNo = (int)(xy.j/1000000);
    CVect3 xyz(xy.i, xy.j-stripNo*1000000-500000, xy.k), BL=IGKPCore(xyz);
    BL.j += (stripNo*6-3)*DEG;
    return BL;
}

//***************************  class CEarth  *********************************/
CEarth::CEarth(double a0, double f0, double g0)
{
	Init(a0, f0, g0);
}

void CEarth::Init(double a0, double f0, double g0)
{
	a = a0;	f = f0; wie = glv.wie; 
	b = (1-f)*a;
	e = sqrt(a*a-b*b)/a;	e2 = e*e;
	gn = O31;  pgn = 0;  pos0=One31;
	Update(O31);
}

void CEarth::Update(const CVect3 &pos, const CVect3 &vn, int isMemsgrade)
{
	this->pos = pos;  this->vn = vn;
	CVect3 dpos; VSUB(dpos, pos, pos0);
//	if(dpos.i>10.0/RE||dpos.i<-10.0/RE || dpos.k>1.0||dpos.k<-1.0)  // fast
	{
		sl = sin(pos.i), cl = cos(pos.i), tl = sl/cl;
		double sq = 1-e2*sl*sl, sq2 = sqrt(sq);
		RMh = a*(1-e2)/sq/sq2+pos.k;	f_RMh = 1.0/RMh;
		RNh = a/sq2+pos.k;    clRNh = cl*RNh;  f_RNh = 1.0/RNh; f_clRNh = 1.0/clRNh;
		VEQU(pos0, pos);
	}
	if(isMemsgrade) {
//		wnie.i = 0.0,			wnie.j = wie*cl,		wnie.k = wie*sl;
//		wnen.i = -vn.j*f_RMh,	wnen.j = vn.i*f_RNh,	wnen.k = wnen.j*tl;
		wnin = wnie = wnen = O31;
		sl2 = sl*sl;
		gn.k = -( glv.g0*(1+5.27094e-3*sl2)-3.086e-6*pos.k );
		gcc = pgn ? *pgn : gn;
	}
	else {
		wnie.i = 0.0,			wnie.j = wie*cl,		wnie.k = wie*sl;
		wnen.i = -vn.j*f_RMh,	wnen.j = vn.i*f_RNh,	wnen.k = wnen.j*tl;
//		wnin = wnie + wnen;
		wnin.i=/*wnie.i+*/wnen.i, wnin.j=wnie.j+wnen.j, wnin.k=wnie.k+wnen.k;
		sl2 = sl*sl, sl4 = sl2*sl2;
		gn.k = -( glv.g0*(1+5.27094e-3*sl2+2.32718e-5*sl4)-3.086e-6*pos.k );
		gcc = pgn ? *pgn : gn;
//		gcc -= (wnie+wnin)*vn;
		wnnin.i=wnin.i, wnnin.j=wnie.j+wnin.j, wnnin.k=wnie.k+wnin.k;
		gcc.i -= crosI(wnnin,vn), gcc.j -= crosJ(wnnin,vn), gcc.k -= crosK(wnnin,vn);
	}
}


CVect3 CEarth::vn2dpos(const CVect3 &vn, double ts) const
{
	return CVect3(vn.j*f_RMh, vn.i*f_clRNh, vn.k)*ts;
}

void CEarth::vn2dpos(CVect3 &dpos, const CVect3 &vn, double ts) const
{
	dpos.i=vn.j*f_RMh*ts, dpos.j=vn.i*f_clRNh*ts, dpos.k=vn.k*ts;
}

double pos2g(const CVect3 &pos)
{
	double sl=sin(pos.i), sl2=sl*sl, sl4=sl2*sl2;
	return G0*(1+5.27094e-3*sl2+2.32718e-5*sl4)-3.086e-6*pos.k;
}

CVect3 pos2wnie(const CVect3 &pos)
{
	return CVect3(0.0, WIE*cos(pos.i), WIE*sin(pos.i));
}

//***************************  class CIMU  *********************************/
CIMU::CIMU(void)
{
	nSamples = 1;
	Reset();
	phim = dvbm = wmm = vmm = swmm = svmm = wm_1 = vm_1 = O31;
	pSf = NULL; pTempArray = NULL; pKga = pgSens = pgSens2 = pgSensX = NULL; pKapn = pKa2 = pTau = NULL; prfu = NULL;
	plv = NULL; pCba = NULL;   tk = tGA = 0.0;
	Sfg = Sfa = One31;  Kg = Ka = Cba = iTCba = I33;
	eb = db = lvx = lvy = lvz = Kapn = Ka2 = Q11 = Q12 = Q13 = Q21 = Q22 = Q23 = Q31 = Q32 = Q33 = O31;  SSx = SSy = SSz = O33;
}

void CIMU::Reset(void)
{
	preFirst = onePlusPre = preWb = true;  swmm = svmm = wm_1 = vm_1 = O31;  smmT = 0.0;
	compensated = false;
}

void CIMU::SetRFU(const char *rfu0)
{
	for(int i=0; i<3; i++) rfu[i]=rfu0[i];
	prfu = rfu;
}

void CIMU::SetSf(const CVect3 &Sfg0, const CVect3 &Sfa0)
{
	Sfg = Sfg0;  Sfa = Sfa0;
	pSf = &Sfg;
}

void CIMU::SetTemp(double *tempArray0, int type)
{
	pTempArray = tempArray0;  iTemp = 0;
	double *p=&Kg.e00, *p1=tempArray0;
	for(int k=0; k<37; k++,p++,p1+=5) *p = *p1;
	pKga = &Kg;  if(type>1) pKa2 = &Ka2;  if(type>2) plv = &lvx;
}

void CIMU::SetKga(const CMat3 &Kg0, const CVect3 eb0, const CMat3 &Ka0, const CVect3 &db0)
{
	Kg = Kg0; eb = eb0; Ka = Ka0; db = db0;
	pKga = &Kg;
}

void CIMU::SetgSens(const CMat3 &gSens0, const CMat3 &gSens20, const CMat3 &gSensX0)
{
	gSens = gSens0;	pgSens = &gSens;
	if(&gSens20!=&O33) { gSens2 = gSens20;	pgSens2 = &gSens2; }
	if(&gSensX0!=&O33) { gSensX = gSensX0;	pgSensX = &gSensX; }
}

void CIMU::SetKa2(const CVect3 &Ka20)
{
	Ka2 = Ka20;	pKa2 = &Ka2;
}

void CIMU::SetKapn(const CVect3 &Kapn0)
{
	Kapn = Kapn0;	pKapn = &Kapn;
}

void CIMU::SetLvtGA(const CVect3 &lvx0, const CVect3 &lvy0, const CVect3 &lvz0, double tGA0)
{
	lvx = lvx0; lvy = lvy0; lvz = lvz0; tGA = tGA0;
	plv = &lvx;
}

void CIMU::SetTau(const CVect3 &taux0, const CVect3 &tauy0, const CVect3 &tauz0)
{
	Taux = taux0;	Tauy = tauy0;	Tauz = tauz0;	pTau = &Taux;
}

void CIMU::SetCba(const CMat3 &Cba0)
{
	Cba = Cba0;  iTCba = inv(~Cba);  pCba = &Cba;
	CMat3 U = inv(~Cba);
	CVect3 V1=Cba.GetClm(0), V2=Cba.GetClm(1), V3=Cba.GetClm(2);
    Q11 = U.e00*V1, Q12 = U.e01*V2, Q13 = U.e02*V3,
    Q21 = U.e10*V1, Q22 = U.e11*V2, Q23 = U.e12*V3,
    Q31 = U.e20*V1, Q32 = U.e21*V2, Q33 = U.e22*V3;
}

double CIMU::GetMeanwf(CVect3 &wib, CVect3 &fsf, BOOL reset)
{
	double mT=smmT;
	wib = swmm*(1.0/smmT);  fsf = svmm*(1.0/smmT);
	if(reset) { swmm=svmm=O31; smmT=0.0; }
	return mT;
}

double CIMU::Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts)
{
	static double conefactors[5][4] = {				// coning coefficients
		{2./3},										// 2
		{9./20, 27./20},							// 3
		{54./105, 92./105, 214./105},				// 4
		{250./504, 525./504, 650./504, 1375./504}	// 5
		};
	int i;
	double *pcf = conefactors[nSamples-2];
	CVect3 cm(0.0), sm(0.0), wm[6], vm[6], vmma(0.0); wmm=O31, vmm=O31;

	psinsassert(nSamples>0 && nSamples<6);
	for(i=0; i<nSamples; i++) { wm[i]=pwm[i]; vm[i]=pvm[i]; }
	if(pSf)
	{
		for(i=0; i<nSamples; i++) { wm[i] = dotdiv(wm[i], Sfg);  vm[i] = dotdiv(vm[i], Sfa); }
	}
	if(pKa2||pKapn)
	{
		for(i=0; i<nSamples; i++) { vmma+=vm[i]; }
	}
	if(pKga)
	{
		for(i=0; i<nSamples; i++) { wm[i] = Kg*wm[i] - eb*ts;  vm[i] = Ka*vm[i] - db*ts;  }
	}
	this->nSamples = nSamples; nts=nSamples*ts; _nts=1.0/nts; tk+=nts;
	if(nSamples==1 && onePlusPre)  // one-plus-previous sample
	{
		if(preFirst) { wm_1=wm[0]; vm_1=vm[0]; preFirst=false; }
		cm = 1.0/12*wm_1;
		sm = 1.0/12*vm_1;
	}
	for(i=0; i<nSamples-1; i++)
	{
		cm += pcf[i]*wm[i];
		sm += pcf[i]*vm[i];
		wmm += wm[i];
		vmm += vm[i];
	}
	wm_1=wm[i];  vm_1=vm[i];
	wmm += wm[i];
	vmm += vm[i];
	phim = wmm + cm*wm[i];
	dvbm = vmm + 1.0/2*wmm*vmm + (cm*vm[i]+sm*wm[i]);
	if(nSamples==1 && compensated) 
	{
		phim = wmm;
		dvbm = vmm;
	}
	CVect3 _phim(0.0), _dvbm(0.0);
	if(pTempArray)
	{
		double *p=&pTempArray[iTemp*5];
		*(&Kg.e00+iTemp) = p[0] + polyval(&p[1], 3, Temp);
		if(++iTemp==40) iTemp=0;  // (&tGA-&Kg.e00)==37
	}
	if(pgSens) {
		_phim.i += gSens.e00*dvbm.i+gSens.e01*dvbm.j+gSens.e02*dvbm.k;   // gSens.eij in (rad/s)/(m/ss)
		_phim.j += gSens.e10*dvbm.i+gSens.e11*dvbm.j+gSens.e12*dvbm.k;
		_phim.k += gSens.e20*dvbm.i+gSens.e21*dvbm.j+gSens.e22*dvbm.k;
	}
	if(pgSens2) {
		double fx2_Ts = dvbm.i*dvbm.i*_nts,  fy2_Ts = dvbm.j*dvbm.j*_nts,  fz2_Ts = dvbm.k*dvbm.k*_nts;
		_phim.i += gSens2.e00*fx2_Ts+gSens2.e01*fy2_Ts+gSens2.e02*fz2_Ts;   // gSens2.eij in (rad/s)/(m/ss)^2
		_phim.j += gSens2.e10*fx2_Ts+gSens2.e11*fy2_Ts+gSens2.e12*fz2_Ts;
		_phim.k += gSens2.e20*fx2_Ts+gSens2.e21*fy2_Ts+gSens2.e22*fz2_Ts;
	}
	if(pgSensX) {
		double fxy_Ts = dvbm.i*dvbm.j*_nts,  fyz_Ts = dvbm.j*dvbm.k*_nts,  fzx_Ts = dvbm.k*dvbm.i*_nts;
		_phim.i += gSensX.e00*fxy_Ts+gSensX.e01*fyz_Ts+gSensX.e02*fzx_Ts;   // gSensX.eij in (rad/s)/(m/ss)^2
		_phim.j += gSensX.e10*fxy_Ts+gSensX.e11*fyz_Ts+gSensX.e12*fzx_Ts;
		_phim.k += gSensX.e20*fxy_Ts+gSensX.e21*fyz_Ts+gSensX.e22*fzx_Ts;
	}
	if(pTau) {
//		_phim.i += (dvbm.i*Taux.i+dvbm.j*Taux.j+dvbm.k*Taux.k)*phim.i*_nts;
//		_phim.j += (dvbm.i*Tauy.i+dvbm.j*Tauy.j+dvbm.k*Tauy.k)*phim.j*_nts;
//		_phim.k += (dvbm.i*Tauz.i+dvbm.j*Tauz.j+dvbm.k*Tauz.k)*phim.k*_nts;
		CVect3 wXf=wmm*vmm;
		_phim.i += dot(Taux, wXf)*_nts;
		_phim.j += dot(Tauy, wXf)*_nts;
		_phim.k += dot(Tauz, wXf)*_nts;
	}
	if(pKapn) {
		_dvbm += iTCba*dotmul(Kapn,abs(vmma));  // Kapn in (m/ss)/(m/ss)=(m/s)/(m/s)=no unit
	}
	if(pKa2) {
		vmma.i = Ka2.i*vmma.i*vmma.i*_nts;   // Ka2.i in (m/ss)/(m/ss)^2
		vmma.j = Ka2.j*vmma.j*vmma.j*_nts;
		vmma.k = Ka2.k*vmma.k*vmma.k*_nts;
		_dvbm += iTCba*vmma;
	}
	if(plv) {
		if(preWb) { wb_1=phim*_nts, preWb=false; }
		CVect3 wb=phim*_nts, dwb=(wb-wb_1)*_nts;  wb_1=wb;
		CMat3 W=askew(dwb)+pow(askew(wb),2);
		CVect3 fL;
		if(pCba) {
			SSx = CMat3(Q11*W, Q21*W, Q31*W);  SSy = CMat3(Q12*W, Q22*W, Q32*W);  SSz = CMat3(Q13*W, Q23*W, Q33*W);
			fL = SSx*lvx + SSy*lvy + SSz*lvz;
		}
		else {
			fL = CVect3(dot(*(CVect3*)&W.e00,lvx), dot(*(CVect3*)&W.e10,lvy), dot(*(CVect3*)&W.e20,lvz));
		}
		_dvbm += fL*nts + tGA*(wb*dvbm);
	}
	wmm -= _phim;	  vmm -= _dvbm;
	phim -= _phim;	  dvbm -= _dvbm;
	swmm += wmm;     svmm += vmm;  smmT += nts;
	if(prfu) { IMURFU(&wmm, &vmm, 1, prfu);  IMURFU(&phim, &dvbm, 1, prfu); IMURFU(&swmm, &svmm, 1, prfu); }
	return tk;
}

void IMURFU(CVect3 *pwm, int nSamples, const char *str)
{
	if(str[0]=='X') return;
	for(int n=0; n<nSamples; n++)
	{
		CVect3 tmpwm;
		double *pw=(double*)&pwm[n].i;
		for(int i=0; i<3; i++,pw++)
		{
			switch(str[i])
			{
			case 'R':  tmpwm.i= *pw;  break;
			case 'L':  tmpwm.i=-*pw;  break;
			case 'F':  tmpwm.j= *pw;  break;
			case 'B':  tmpwm.j=-*pw;  break;
			case 'U':  tmpwm.k= *pw;  break;
			case 'D':  tmpwm.k=-*pw;  break;
			}
		}
		pwm[n] = tmpwm;
	}
}

void IMURFU(CVect3 *pwm, CVect3 *pvm, int nSamples, const char *str)
{
	if(str[0]=='X') return;
	IMURFU(pwm, nSamples, str);
	IMURFU(pvm, nSamples, str);
}

void IMUStatic(CVect3 &wm, CVect3 &vm, CVect3 &att0, CVect3 &pos0, double ts)
{
	CEarth eth;
	eth.Update(pos0);
	CMat3 Cbn=~a2mat(att0);
	wm = Cbn*eth.wnie*ts;	vm = -Cbn*eth.gn*ts;
}

CMat lsclbt(CMat &wfb, CMat &wfn)
{
	CMat A(wfb.row, 4, 0.0), Kga_edb(4,3,0.0);
	CVect Y(wfb.row), X;
	for(int k=0; k<3; k++)
	{
		for(int i=0; i<wfb.row; i++)
		{
			A.SetRow(i, CVect(4, wfb(i,0), wfb(i,1), wfb(i,2), -1.0));
			Y(i) = wfn(i,k);
		}
		X = inv4((~A)*A)*((~A)*Y);
		Kga_edb(k,0)=X(0), Kga_edb(k,1)=X(1), Kga_edb(k,2)=X(2), Kga_edb(3,k)=X(3);
	}
	return Kga_edb;
}

//**************************  class CIMUInc  ********************************/
CIMUInc::CIMUInc(double gScale, double aScale)
{
	Init();
}

void CIMUInc::Init(double gScale, double aScale)
{
	diGx=diGy=diGz=diAx=diAy=diAz=0;
	iTotalGx0=iTotlaGy0=iTotalGz0=iTotalAx0=iTotalAy0=iTotalAz0=0;
	iTotalGx=iTotalGy=iTotalGz=iTotalAx=iTotalAy=iTotalAz=0;
	fTotalGx=fTotalGy=fTotalGz=fTotalAx=fTotalAy=fTotalAz=0.0;
	this->gScale = gScale,  this->aScale = aScale;
}

void CIMUInc::Update(const CVect3 &wm, const CVect3 &vm)
{
#define MAX_NN  2147483648.0  // 2^31
#define MAX_PP  (MAX_NN-1.0)
	fTotalGx += wm.i/gScale, fTotalGy += wm.j/gScale, fTotalGz += wm.k/gScale;
	fTotalAx += vm.i/aScale, fTotalAy += vm.j/aScale, fTotalAz += vm.k/aScale;
	double *pfTotal=&fTotalGx;  int *pdi = &diGx, *piTotal0 = &iTotalGx0, *piTotal = &iTotalGx;
	for(int i=0; i<6; i++,pfTotal++,pdi++,piTotal0++,piTotal++) {
		if(*pfTotal>=MAX_PP)		*pfTotal -= 2.0*MAX_NN;
		else if(*pfTotal<=-MAX_NN) 	*pfTotal += 2.0*MAX_NN;
		*piTotal = (int)(*pfTotal);
		*pdi = *piTotal - *piTotal0;
		if(*pdi>100000000)			*pdi -= 2147483648;
		else if(*pdi<-100000000)	*pdi += 2147483648;
		*piTotal0 = *piTotal;
	}
}

//**************************  class CIMUInv  ********************************/
CIMUInv::CIMUInv(const CVect3 &att00, const CVect3 &vn00, const CVect3 &pos00, double ts0, double tk0)
{
	vn0 = vn00;  pos0 = pos00;  att = att00;  vn = vn0;  pos = pos0;
	Cbn0 = ~a2mat(att00);
	eth.Update(pos, vn);
	ts = ts0;  tk = tk0;
	isFirst = TRUE;
}

void CIMUInv::Update(const CVect3 &att1, const CVect3 &pos1)
{
	CVect3 vn1 = pp2vn(pos1, pos0, ts, &eth);  vn1 = vn0+(vn1-vn0)*2;
	eth.Update((pos0+pos1)/2, (vn0+vn)/2);
	CMat3 Cnb1 = a2mat(att1);
	CVect3 phim = m2rv(Cbn0*rv2m(eth.wnin*ts)*Cnb1);
	if(isFirst) wm0=phim;
    wm = inv(I33+askew(wm0/12.0))*phim;
    CVect3 dvbm = Cbn0 * (rv2q(eth.wnin*ts*2) * (vn1-vn0-eth.gcc*ts));
	if(isFirst) { vm0=dvbm; isFirst=FALSE; }
    vm = inv(I33+askew(wm/2.0+wm0/12.0))*(dvbm-vm0*wm/12.0);
	wm0 = wm;  vm0 = vm;
	Cbn0 = ~Cnb1;  vn0 = vn1;  pos0 = pos1;
	att = att1;  vn = vn1;  pos = pos1;
	tk += ts;
}

//***************************  class CSINS  *********************************/
CSINS::CSINS(double yaw0, const CVect3 &pos0, double tk0)
{
	Init(a2qua(CVect3(0,0,yaw0)), O31, pos0, tk0);
}

CSINS::CSINS(const CVect3 &att0, const CVect3 &vn0, const CVect3 &pos0, double tk0)
{
	Init(a2qua(att0), vn0, pos0, tk0);
}

CSINS::CSINS(const CQuat &qnb0, const CVect3 &vn0, const CVect3 &pos0, double tk0)
{
	Init(qnb0, vn0, pos0, tk0);
}

void CSINS::Init(const CVect3 &att0, const CVect3 &vn0, const CVect3 &pos0, double tk0)
{
	Init(a2qua(att0), vn0, pos0, tk0);
}

void CSINS::Init(const CQuat &qnb0, const CVect3 &vn0, const CVect3 &pos0, double tk0)
{
	tk = tpps = tk0;  ts = nts = 1.0;  dist = -EPS;
	velMax = 400.0; hgtMin = -RE*0.01, hgtMax = -hgtMin; latMax = 85.0*DEG; afabar = 0.1;
	qnb = qnb0;	vn = vn0, pos = pos0;
	Kg = Ka = I33; eb = db = Ka2 = O31;
	Maa = Mav = Map = Mva = Mvv = Mvp = Mpv = Mpp = O33;
	SetTauGA(CVect3(INF),CVect3(INF));
	CVect3 wib(0.0), fb=(~qnb)*CVect3(0,0,glv.g0);
	lvr = an = anbar = webbar = O31;
	isOpenloop = isMemsgrade = isNocompasseffect = isOutlever = iReverse = 0;
	Cbn = I33;
	Update(&wib, &fb, 1, 1.0); imu.preFirst = 1;
	tk = tk0;  ts = nts = 1.0; qnb = qnb0;	att=q2att(qnb), vn = vn0, pos = pos0;
	mmwb = mmfb = CMaxMin(100);
	mvn = mvnmax = mvnmin = vn; mvni = O31; mvnt = mvnT = lvlT = 0.0; mvnk = 0;
	etm(); lever(); Extrap();
}

void CSINS::SetTauGA(const CVect3 &tauG, const CVect3 &tauA)
{
	if(tauG.i>EPS) {
		tauGyro = tauG;
		_betaGyro.i = tauG.i>INFp5 ? 0.0 : -1.0/tauG.i;   // Gyro&Acc inverse correlation time for AR(1) model
		_betaGyro.j = tauG.j>INFp5 ? 0.0 : -1.0/tauG.j;
		_betaGyro.k = tauG.k>INFp5 ? 0.0 : -1.0/tauG.k;
	}
	if(tauA.i>EPS) {
		tauAcc = tauA;
		_betaAcc.i  = tauA.i>INFp5 ? 0.0 : -1.0/tauA.i;
		_betaAcc.j  = tauA.j>INFp5 ? 0.0 : -1.0/tauA.j;
		_betaAcc.k  = tauA.k>INFp5 ? 0.0 : -1.0/tauA.k;
	}
}

void CSINS::Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts)
{
	this->ts = ts;  nts = nSamples*ts;	tk += nts;
	double nts2 = nts/2, _nts=1.0/nts;
	if(isMemsgrade) {
		imu.Update(pwm, pvm, nSamples, ts);
		imu.phim = Kg*imu.phim - eb*nts; imu.dvbm = Ka*imu.dvbm - db*nts;  // IMU calibration
		if(!isOpenloop) eth.Update(pos,O31,1);
		wib = imu.phim*_nts; fb = imu.dvbm*_nts;
		web = wib;  webbar = (1-afabar)*webbar + afabar*web;
		wnb = wib;
		fn = qnb*fb;
		an = fn+eth.gcc;  anbar = (1-afabar)*anbar + afabar*an;
		CVect3 vn1 = vn + an*nts;
		pos = pos + eth.vn2dpos(vn+vn1, nts2);	vn = vn1;
		qnb = qnb*rv2q(imu.phim);
		Cnb = q2mat(qnb); att = m2att(Cnb); Cbn = ~Cnb; vb = Cbn*vn;
	}
	else {
#ifndef PSINS_FAST_CALCULATION
		imu.Update(pwm, pvm, nSamples, ts);
		imu.phim = Kg*imu.phim - eb*nts; imu.dvbm = Ka*imu.dvbm - db*nts;  // IMU calibration
		CVect3 vn01 = vn+an*nts2, pos01 = pos+eth.vn2dpos(vn01,nts2);
		if(!isOpenloop) eth.Update(pos01, vn01);
		wib = imu.phim/nts; fb = imu.dvbm/nts;
		web = wib - Cbn*eth.wnie;  webbar = (1-afabar)*webbar + afabar*web;
		wnb = wib - Cbn*eth.wnin;
		fn = qnb*fb;
		an = rv2q(-eth.wnin*nts2)*fn+eth.gcc;  anbar = (1-afabar)*anbar + afabar*an;
		CVect3 vn1 = vn + an*nts;
		pos = pos + eth.vn2dpos(vn+vn1, nts2);	vn = vn1;
		qnb = rv2q(-eth.wnin*nts)*qnb*rv2q(imu.phim);
		Cnb = q2mat(qnb); att = m2att(Cnb); Cbn = ~Cnb; vb = Cbn*vn;
#else  // fast
		imu.Update(pwm, pvm, nSamples, ts);
//		imu.phim = Kg*imu.phim - eb*nts; imu.dvbm = Ka*imu.dvbm - db*nts;  // IMU calibration
		AXbt(imu.phim, Kg, imu.phim, eb, -nts);  AXbt(imu.dvbm, Ka, imu.dvbm, db, -nts);
//		CVect3 vn01 = vn+an*nts2, pos01 = pos+eth.vn2dpos(vn01,nts2);
		CVect3 vn01,pos01,dpos; VADDf(vn01,vn,an,nts2); eth.vn2dpos(dpos,vn01,nts2); VADD(pos01,pos,dpos);
		if(!isOpenloop) eth.Update(pos01, vn01);
//		wib = imu.phim*_nts; fb = imu.dvbm*_nts;
		VMULf(wib, imu.phim, _nts);  VMULf(fb, imu.dvbm, _nts);
//		web = wib - Cbn*eth.wnie;  webbar = (1-afabar)*webbar + afabar*web;
		CVect3 vtmp; mul(vtmp, Cbn, eth.wnie); VSUB(web, wib, vtmp); double b=1-afabar; VADDff(webbar, webbar, b, web, afabar);
//		wnb = wib - Cbn*eth.wnin;
		mul(vtmp, Cbn, eth.wnin); VSUB(wnb, wib, vtmp);
//		fn = qnb*fb;
		mul(fn, qnb, fb);
//		an = rv2q(-eth.wnin*nts2)*fn+eth.gcc;  anbar = (1-afabar)*anbar + afabar*an;
		CVect3 wnint, fnn; double tn=-nts2; VMULf(wnint,eth.wnin,tn);
		CQuat q; rv2q(q, wnint); mul(fnn, q, fn);
		VADD(an, fnn, eth.gcc);  VADDff(anbar, anbar, b, an, afabar);
//		CVect3 vn1 = vn + an*nts;
		CVect3 vn1; VADDf(vn1, vn, an, nts);
//		pos = pos + eth.vn2dpos(vn+vn1, nts2);	vn = vn1;
		VADD(vn01, vn, vn1);  eth.vn2dpos(dpos, vn01, nts2);  VADD(pos, pos, dpos);  VEQU(vn, vn1);
//		qnb = rv2q(-eth.wnin*nts)*qnb*rv2q(imu.phim);
		tn=-nts; VMULf(wnint,eth.wnin,tn);	rv2q(q, wnint);  mul(qnb, q, qnb);  rv2q(q, imu.phim);  mul(qnb, qnb, q); 
//		Cnb = q2mat(qnb); att = m2att(Cnb); Cbn = ~Cnb; vb = Cbn*vn;
		q2mat(Cnb, qnb);  m2att(att, Cnb);  _TT(Cbn, Cnb);  mul(vb, Cbn, vn);
#endif		
	}
	psinsassert(pos.i<85.0*DEG && pos.i>-85.0*DEG);
	if(vn.i>velMax) vn.i=velMax; else if(vn.i<-velMax) vn.i=-velMax;
	if(vn.j>velMax) vn.j=velMax; else if(vn.j<-velMax) vn.j=-velMax;
	if(vn.k>velMax) vn.k=velMax; else if(vn.k<-velMax) vn.k=-velMax;
	if(pos.i>latMax) pos.i=latMax; else if(pos.i<-latMax) pos.i=-latMax;
	if(pos.j>PI) pos.j-=_2PI; else if(pos.j<-PI) pos.j+=_2PI;
	if(pos.k>hgtMax) pos.k=hgtMax; else if(pos.k<hgtMin) pos.k=hgtMin;
	if(mvnT>EPS) {   // calculate max/min/mean-vn within mvnT
		if(mvnk==0) {
			mvnmax = mvnmin = vn;
			mvni = O31;  mvnt = 0.0;  mvnCnb0 = Cnb;
		}
		else {
			if(vn.i>mvnmax.i) mvnmax.i=vn.i; else if(vn.i<mvnmin.i) mvnmin.i=vn.i; 
			if(vn.j>mvnmax.j) mvnmax.j=vn.j; else if(vn.j<mvnmin.j) mvnmin.j=vn.j; 
			if(vn.k>mvnmax.k) mvnmax.k=vn.k; else if(vn.k<mvnmin.k) mvnmin.k=vn.k; 
		}
		mvni += vn;  mvnt += nts;  mvnk++;
		if(mvnt>=mvnT) {
			mvn = mvni*(1.0/mvnk);
			mvnk = 0;   // OK if mvnk==0
		}
	}
	if(dist>=0.0)
		dist += sqrt(vn.i*vn.i + vn.j*vn.j + vn.k*vn.k)*nts;  // travel distance ^2
	if(lvlT>=0)
		lvlT += nts;
	mmwb.Update((float)(norm(imu.wmm)*_nts));
	mmfb.Update((float)(norm(imu.vmm)*_nts+eth.gn.k));
}

void CSINS::Extrap(const CVect3 &wm, const CVect3 &vm, double ts)
{
	if(ts<1.0e-6)  // reset
	{
		qnbE = qnb, vnE = vn, posE = pos, attE = att;
	}
	else
	{
		vnE = vnE + qnbE*vm + eth.gcc*ts;
		posE = posE + eth.vn2dpos(vnE,ts);
		qnbE = qnbE*rv2q(wm); attE = q2att(qnbE);
	}
}

void CSINS::Extrap(double extts)
{
	double k = extts/nts;
	vnE = vn + qnb*imu.dvbm*k + eth.gcc*extts;
	posE = pos + eth.vn2dpos(vn,extts);
	attE = q2att(qnb*rv2q(imu.phim*k));
}

void CSINS::lever(const CVect3 &dL, CVect3 *ppos, CVect3 *pvn)
{
	if(&dL!=&O31) lvr = dL;
//	Mpv = CMat3(0,eth.f_RMh,0, eth.f_clRNh,0,0, 0,0,1);
	Mpv.e01=eth.f_RMh, Mpv.e10=eth.f_clRNh, Mpv.e22=1.0;
	CW = Cnb*askew(web), MpvCnb = Mpv*Cnb;
	if(ppos==NULL) {
		posL = pos + MpvCnb*lvr;
		if(pvn==NULL)  vnL = vn + CW*lvr;
	} 
	else {
		*ppos = pos + MpvCnb*lvr;
		if(pvn!=NULL)  *pvn = vn + CW*lvr;
	}
}

void CSINS::lever2(const CVect3 &dL, CVect3 *ppos, CVect3 *pvn, const CVect3 *ppos0, const CVect3 *pvn0)
{
	if(&dL!=&O31) lvr = dL;
	if(ppos0==NULL) ppos0=ppos;
	if(pvn0==NULL) pvn0=pvn;
	Mpv.e01=eth.f_RMh, Mpv.e10=eth.f_clRNh, Mpv.e22=1.0;
	CW = Cnb*askew(web), MpvCnb = Mpv*Cnb;
	if(ppos!=NULL)  *ppos = *ppos0 + MpvCnb*lvr;
	if(pvn!=NULL)  *pvn = *pvn0 + CW*lvr;
}

void CSINS::atss(double *attack, double *sideslip)
{
	CVect3 as=::atss(att, vn);
	*attack = as.i,  *sideslip = as.k;
}

void CSINS::etm(void)
{
	if(isMemsgrade) {
		Mva = askew(fn);
		Mpv = CMat3(0,eth.f_RMh,0, eth.f_clRNh,0,0, 0,0,1);
	}
	else {
		double tl=eth.tl, secl=1.0/eth.cl, secl2=secl*secl, 
			wN=eth.wnie.j, wU=eth.wnie.k, vE=vn.i, vN=vn.j;
		double f_RMh=eth.f_RMh, f_RNh=eth.f_RNh, f_clRNh=eth.f_clRNh, 
			f_RMh2=f_RMh*f_RMh, f_RNh2=f_RNh*f_RNh;
		CMat3 Avn=askew(vn),
			Mp1(0,0,0, -wU,0,0, wN,0,0),
			Mp2(0,0,vN*f_RMh2, 0,0,-vE*f_RNh2, vE*secl2*f_RNh,0,-vE*tl*f_RNh2);
		if(isNocompasseffect)	Maa = O33;
		else					Maa = askew(-eth.wnin);
		Mav = CMat3(0,-f_RMh,0, f_RNh,0,0, tl*f_RNh,0,0);
		Map = Mp1+Mp2;
		Mva = askew(fn);
		Mvv = Avn*Mav - askew(eth.wnie+eth.wnin);
		Mvp = Avn*(Mp1+Map);
		double scl = eth.sl*eth.cl;
		Mvp.e20 = Mvp.e20-glv.g0*(5.27094e-3*2*scl+2.32718e-5*4*eth.sl2*scl); Mvp.e22 = Mvp.e22+3.086e-6;
		Mpv = CMat3(0,f_RMh,0, f_clRNh,0,0, 0,0,1);
		Mpp = CMat3(0,0,-vN*f_RMh2, vE*tl*f_clRNh,0,-vE*secl*f_RNh2, 0,0,0);
	}
}

void CSINS::AddErr(const CVect3 &phi, const CVect3 &dvn, const CVect3 &dpos)
{
	qnb -= -phi;
	vn += dvn;
	pos += CVect3(dpos.i*eth.f_RMh, dpos.j*eth.f_clRNh, dpos.k);  // NOTE: dpos in meter
}

void CSINS::AddErr(double phiU, const CVect3 &dvn, const CVect3 &dpos)
{
	AddErr(CVect3(0,0,phiU), dvn, dpos);
}

void CSINS::Leveling(int flag)
{
	if(flag==0) {
		lvlVn0 = vn;  lvlT = EPS;  // record
	}
	else if(flag>=1)
	{
		CVect3 dvn = vn-lvlVn0;  vn = O31;
		CVect3 phi = CVect3(dvn.j, -dvn.i, 0)/(lvlT*G0);
		qnb -= phi;   // rectify
		if(flag==2)  db.k += dvn.k/lvlT;
		lvlT = -1.0;  // stop
	}
}

void CSINS::Reverse(void)
{
	eth.wie = -eth.wie;  vn = -vn; vnL = -vnL; eb = -eb;
	iReverse = -1;
}

BOOL CSINS::isPPS(double pps)
{
	tpps = pps*(int)((tk/pps));
	return (tk+nts)>tpps&&(tk-nts)<tpps ? 1 : 0;
}

void CSINS::DebugStop(double t1, int absT, int ext)
{
	static double dst0=-INF;
	if(dst0<-INFp5) dst0=tk;
	double t=tk-dst0*absT;  // absT=0 for absolute time
	if(t1<t && t<t1+3*nts) {
		if(ext==0)
			t1 = tk-dst0*absT;  // please place breakpoint here
		else
			exit(0);  // for exit
	}
}

//***************************  class CDR  *********************************/
CDR::CDR(void)
{
}

void CDR::Init(const CSINS &sins, const CVect3 &kappa)
{
	Init(sins.att, sins.pos, kappa, sins.tk);
	vn = sins.vn;  velPre = norm(vn);
}

void CDR::Init(const CVect3 &att0, const CVect3 &pos0, const CVect3 &kappa, double tk0)
{
	Kod = kappa.j;
	Cbo = a2mat(CVect3(kappa.i,0.0,kappa.k));
	qnb = a2qua(att0);  Cnb = q2mat(qnb); att = m2att(Cnb);
	vn = O31;	pos = pos0;  eth.Update(pos, vn);
	velPre = 0.0;  velMax=50.0; velMin=-3.0;  afa = 0.1;
	tk = tk0;  dist = 0.0;
	SetGCK(10.0);
}

void CDR::Update(const CVect3 &wm, double dS, double ts, const CVect3 &vm)
{
	tk += ts;  dS *= Kod; dist += dS;
	double vel = dS/ts;
	if(vel>velMax||vel<velMin) vel=velPre;  // if abnormal, keep last velocity
	CVect3 vnk = Cnb*(Cbo*CVect3(0,vel,0));  velPre=vel;
	eth.Update(pos, vnk);
	vn = (1-afa)*vn + afa*vnk;  // AR(1) vel filter
	pos = pos + eth.vn2dpos(vnk, ts); 
	qnb = rv2q(-eth.wnin*ts)*qnb*rv2q(wm);
	Cnb = q2mat(qnb); att = m2att(Cnb);
	if(&vm!=&O31) {
		Leveling(vm, ts);
	}
}

void CDR::SetGCK(double Td)
{
	double xi=0.707, xi2=xi*xi, ws2=G0/RE, sigma=_2PI*xi/(Td*sqrt(1.0-xi2)), sigma2=sigma*sigma;
    gck1 = 3.0*sigma; 
    gck2 = sigma2*(2.0+1.0/xi2)/ws2-1.0; 
    gck3 = sigma2*sigma/(G0*xi2);
	wnc = vni = dpos = O31;
}

void CDR::Leveling(const CVect3 &vm, double ts)
{
	CVect3 fn=qnb*vm/ts;
	double dVE = vni.i - vn.i;     // vni: inertial vel;  vn: ref vel
	vni.i = vni.i + (fn.i-gck1*dVE)*ts;
	dpos.i = dpos.i + dVE*gck3*ts;
	wnc.j = dVE*(1+gck2)/RE + dpos.i;
	double dVN = vni.j - vn.j;
	vni.j = vni.j + (fn.j-gck1*dVN)*ts;
	dpos.j = dpos.j + dVN*gck3*ts;
	wnc.i = -dVN*(1+gck2)/RE - dpos.j;
	qnb = rv2q(-wnc*ts)*qnb;
}

CVect3 CDR::Calibrate(const CVect3 &pos0, const CVect3 &pos1, const CVect3 &pos1DR, double dist0)
{
	CVect3 dpos=pos1-pos0, xyz, xyzDR;
	xyz = CVect3(dpos.j*eth.clRNh, dpos.i*eth.RMh, dpos.k);
	dpos=pos1DR-pos0;
	xyzDR = CVect3(dpos.j*eth.clRNh, dpos.i*eth.RMh, dpos.k);
	if(dist0<-EPS) dist0=-norm(xyz);
	else if(dist0<1.0) dist0=norm(xyz);
	double kod = norm(xyz)/norm(xyzDR);
	double dpitch = (xyzDR.k-xyz.k)/dist0;
	double dyaw=crossXY(xyz,xyzDR)/normXY(xyz)/normXY(xyzDR);
	return CVect3(dpitch, kod, dyaw);
}

//***************************  class CCNS  *********************************/
CCNS::CCNS(void)
{
	SetdT();
	Setxyp(0.0, 0.0);
	TT = era = gmst = gast = eps = dpsi = deps = 0.0;  CP = CN = CW = Cie = I33;
}

void CCNS::SetdT(double dUT1, double dTAI)
{
	this->dUT1 = dUT1,  this->dTAI = dTAI;  // dUT1=UT1-UTC, dTAI=TAI-UTC
	dTT = dTAI+32.184;
}

void CCNS::Setxyp(double xp, double yp)
{
	xp *= glv.sec;  yp *= glv.sec;		// inputs xp,yp are in arcsec
	double xyp=xp-yp;
	CW = CMat3( 1-xp*xp/2,	0.0,			xp,			// polar motion matrix, C^e0_e
				0.0,		1-yp*yp/2,		-yp, 
				-xp,		yp,				1-xyp*xyp/2 );
//	CW = Rot(xp,'y')*Rot(yp,'x');
}

double CCNS::JD(int year, int month, int day, double hour)
{
    if(month>2) { month=month+1; }  else { year=year-1; month=month+13; }
    double d = (int)(365.25*(year+4716))+(int)(30.6001*month)+day - 1524.5;
    int ja = (int)(0.01*year);
    d += (2-ja+(int)(0.25*ja));
	return d+hour/24.0;
}

CMat3 CCNS::Precmat(double TT)
{
	double T2 = TT*TT;  // TT ~= TDB
    double zeta  = 2306.2181*TT+0.301880*T2;
    double theta = 2004.3109*TT-0.426650*T2;
    double z     = 2306.2181*TT+1.094687*T2;
    return CP = Rot(z*glv.sec,'z') * Rot(-theta*glv.sec,'y') * Rot(zeta*glv.sec,'z');
}

void CCNS::Equinox(double TT)
{
	double T2 = TT*TT;  // TT ~= TDB
    eps  = (84381.448-46.8150*TT)*glv.sec;
    double lp = (1287104.79305+129596581.0481*TT-0.5532*T2)*glv.sec;
    double F  = (335779.526232+1739527262.8478*TT-12.7512*T2)*glv.sec;
    double D  = (1072260.70369+1602961601.2090*TT-6.3706*T2)*glv.sec;
    double Om = (450160.398036-6962890.5431*TT+7.4722*T2 )*glv.sec;
    dpsi = ( -17.1996*sin(Om)  -1.3187*sin(2*F-2*D+2*Om)  -0.2274*sin(2*F+2*Om)
               +0.2062*sin(2*Om)  +0.1426*sin(lp)   )*glv.sec;
    deps = ( 9.2025*cos(Om)  +0.5736*cos(2*F-2*D+2*Om) )*glv.sec;
}

CMat3 CCNS::Nutmat(double TT)
{
	Equinox(TT);  // TT ~= TDB
    return CN = Rot(eps+deps,'x') * Rot(dpsi,'z') * Rot(-eps,'x');
}

double CCNS::GAST(double jd, double s)
{
	// ERA
	double UT1 = jd-2451545.0 + (s+dUT1)/86400.0;  // 's' is UTC seconds within a day
	era = 4.894961212823756 + 6.300387486754831*UT1; //	era = _2PI*(0.7790572732640 + 1.00273781191135448*UT1);
//	era = fmod(era, _2PI);	if(era<0) era+=_2PI;
	// GMST
	TT = ((jd-2451545.0) + (s+dTT)/86400.0)/36525;
    gmst = era + (4612.15739966*TT+1.39667721*TT*TT)*glv.sec;
//	gmst = fmod(gmst, _2PI);  if(gmst<0) gmst+=_2PI;
	// GAST
	Equinox(TT);
	gast = gmst + dpsi*cos(eps+deps);
	gast = fmod(gast, _2PI);	if(gast<0) gast+=_2PI;
	return gast;
}

CMat3 CCNS::GetCie(double jd, double s)
{
    GAST(jd, s);  // 's' is UTC seconds within a day
	CN = Rot(eps+deps,'x') * Rot(dpsi,'z') * Rot(-eps,'x');
    return Cie = (~(CN*Precmat(TT))) * Rot(gast,'z');
}

CMat3 CCNS::GetCns(const CQuat &qis, const CVect3 &pos, double t, const CMat3 &Cbs)
{
	CMat3 Cie1 = Cie * Rot(glv.wie*t, 'Z');
	Cns = ~(q2mat(~qis) * Cie1 * pos2Cen(pos));
	if(&Cbs!=&I33) Cns = Cns*(~Cbs);
	return Cns;
}

//***************************  class CAVPInterp  *********************************/
CAVPInterp::CAVPInterp(void)
{
}

void CAVPInterp::Init(const CSINS &sins, double ts, BOOL islever, int num)
{
	if(islever)
		Init(sins.att, sins.vnL, sins.posL, ts, num);
	else
		Init(sins.att, sins.vn, sins.pos, ts, num);
}

void CAVPInterp::Init(const CVect3 &att0, const CVect3 &vn0, const CVect3 &pos0, double ts, int num)
{
	psinsassert(num<AVPINUM);
	this->ts = ts;
	ipush = 0;  avpinum = num;
	for(int i=0; i<AVPINUM; i++) { atti[i]=att0, vni[i]=vn0; posi[i]=pos0; }
	att = att0, vn = vn0, pos = pos0;
}

void CAVPInterp::Push(const CSINS &sins, BOOL islever)
{
	if(islever)
		Push(sins.att, sins.vnL, sins.posL);
	else
		Push(sins.att, sins.vn, sins.pos);
}

void CAVPInterp::Push(const CVect3 &attk, const CVect3 &vnk, const CVect3 &posk)
{
	if(++ipush>=avpinum) ipush = 0;
	atti[ipush] = attk; vni[ipush] = vnk; posi[ipush] = posk;
}

int CAVPInterp::Interp(double tpast, int avp)
{
	int res=1, k, k1, k2;
	if(tpast<-avpinum*ts) tpast=-avpinum*ts; else if(tpast>0) tpast=0;
//	if(tpast<-AVPINUM*ts||tpast>0) return (res=0);
	k = (int)(-tpast/ts);
	if((k2=ipush-k)<0) k2 += avpinum;
	if((k1=k2-1)<0)  k1 += avpinum;
	double tleft = -tpast - k*ts;
	if(tleft>0.99*ts)	{
		if(avp&0x1) att=atti[k1]; if(avp&0x2) vn=vni[k1]; if(avp&0x4) pos=posi[k1];
	}
	else if(tleft<0.01*ts)	{
		if(avp&0x1) att=atti[k2]; if(avp&0x2) vn=vni[k2]; if(avp&0x4) pos=posi[k2];
	}
	else	{
		double b=tleft/ts, a=1-b;
		if(avp&0x1)	{ att = b*atti[k1]+a*atti[k2]; if(normInf(att-atti[k1])>10.0*DEG) res=0; }
		if(avp&0x2)	{ vn  = b*vni[k1] +a*vni[k2]; }
		if(avp&0x4)	{ pos = b*posi[k1]+a*posi[k2]; if(fabs(pos.j-posi[k1].j)>1.0*DEG) res=0; }
	}
	return res;
}


#ifdef PSINS_AHRS_MEMS
#pragma message("  PSINS_AHRS_MEMS")

//*********************  class CMahony AHRS  ************************/
CMahony::CMahony(double tau, const CQuat &qnb0)
{
	SetTau(tau);  this->tau = tau;  tau1 = dtau = -1.0;
	qnb = qnb0;
	Cnb = q2mat(qnb);
	exyzInt = O31;  ebMax = One31*glv.dps*5;
	tk = 0.0;
}

void CMahony::SetTau(double tau)	// https://zhuanlan.zhihu.com/p/582694093
{
	double beta = 2.146/tau;
	Kp = 2.0*beta, Ki = beta*beta;
}

void CMahony::SetTau(double tau0, double tau1, double dtau)   // time-varying tau = tau0 -> tau1
{
	SetTau(tau0);
	this->tau = tau0;  this->tau1 = tau1;  this->dtau = dtau;
}

void CMahony::SetWn(double wn, double xi)
{
	Kp = 2.0*xi*wn, Ki = wn*wn;
}

void CMahony::Update(const CVect3 &wm, const CVect3 &vm, double ts, const CVect3 &mag)
{
	double nm;
	CVect3 acc0, mag0, exyz, bxyz, wxyz;

	if(tau<tau1) {
		SetTau(tau);
		tau += (dtau>0.0 ? dtau : ts);
	}
	nm = norm(vm)/ts;  // f (in m/s^2)
	acc0 = nm>0.1 ? vm/(nm*ts) : O31;
	nm = norm(mag);    // mag (in Gauss)
	if(nm>0.1)
	{
		mag0 = mag/nm;
		bxyz = Cnb*mag0;
		bxyz.j = normXY(bxyz); bxyz.i = 0.0;
		wxyz = (~Cnb)*bxyz;
	}
	else
	{
		mag0 = wxyz = O31;
	}
	exyz = *((CVect3*)&Cnb.e20)*acc0 + wxyz*mag0;
	exyzInt += exyz*(Ki*ts);
	CVect3 eb = Kp*exyz + exyzInt;
	if(nm<0.1) {
		CVect3 en=Cnb*eb; en.k=0; eb=(~Cnb)*en;
	}
	qnb *= rv2q(wm-eb*ts);
	Cnb = q2mat(qnb);
	tk += ts;
	if(exyzInt.i>ebMax.i)  exyzInt.i=ebMax.i;  else if(exyzInt.i<-ebMax.i)  exyzInt.i=-ebMax.i;
	if(exyzInt.j>ebMax.j)  exyzInt.j=ebMax.j;  else if(exyzInt.j<-ebMax.j)  exyzInt.j=-ebMax.j;
	if(exyzInt.k>ebMax.k)  exyzInt.k=ebMax.k;  else if(exyzInt.k<-ebMax.k)  exyzInt.k=-ebMax.k;
}

//*********************  class Quat&EKF based AHRS  ************************/
CQEAHRS::CQEAHRS(double ts):CKalman(7,3)
{
	double sts = sqrt(ts);
	Pmax.Set2(2.0,2.0,2.0,2.0, 1000*DPH,1000.0*DPH,1000.0*DPH);
	Pmin.Set2(0.001,0.001,0.001,0.001, 10.0*DPH,10.0*DPH,10.0*DPH);
	Pk.SetDiag2(1.0,1.0,1.0,1.0, 1000.0*DPH,1000.0*DPH,1000.0*DPH);
	Qt.Set2(10.0*glv.dpsh,10.0*glv.dpsh,10.0*glv.dpsh, 10.0*glv.dphpsh,10.0*glv.dphpsh,10.0*glv.dphpsh);
	Rt.Set2(100.0*glv.mg/sts,100.0*glv.mg/sts, 1.0*DEG/sts);  Rt0 = Rt;
	Xk(0) = 1.0;
	Cnb = q2mat(*(CQuat*)&Xk.dd[0]);
}

void CQEAHRS::Update(const CVect3 &gyro, const CVect3 &acc, const CVect3 &mag, double ts)
{
	double q0, q1, q2, q3, wx, wy, wz, fx, fy, fz, mx, my, mz, h11, h12, h21, h22; 
	q0 = Xk.dd[0],		 q1 = Xk.dd[1],		  q2 = Xk.dd[2],		q3 = Xk.dd[3];
	wx = gyro.i*glv.dps, wy = gyro.j*glv.dps, wz = gyro.k*glv.dps; 
	fx = acc.i,			 fy = acc.j,		  fz = acc.k; 
	mx = mag.i,			 my = mag.j,		  mz = mag.k; 
	// Ft
	                0, Ft.dd[ 1] = -wx/2, Ft.dd[ 2] = -wy/2, Ft.dd[ 3] = -wz/2,  Ft.dd[ 4] =  q1/2, Ft.dd[ 5] =  q2/2, Ft.dd[ 6] =  q3/2; 
	Ft.dd[ 7] =  wx/2,                 0, Ft.dd[ 9] =  wz/2, Ft.dd[10] = -wy/2,  Ft.dd[11] = -q0/2, Ft.dd[12] =  q3/2, Ft.dd[13] = -q2/2; 
	Ft.dd[14] =  wy/2, Ft.dd[15] = -wz/2,                 0, Ft.dd[17] =  wx/2,  Ft.dd[18] = -q3/2, Ft.dd[18] = -q0/2, Ft.dd[20] =  q1/2; 
	Ft.dd[21] =  wz/2, Ft.dd[22] =  wy/2, Ft.dd[23] = -wx/2,                 0,  Ft.dd[25] =  q2/2, Ft.dd[26] = -q1/2, Ft.dd[27] = -q0/2; 
	// Hk
    h11 = fx*q0-fy*q3+fz*q2;  h12 = fx*q1+fy*q2+fz*q3;
    h21 = fx*q3+fy*q0-fz*q1;  h22 = fx*q2-fy*q1-fz*q0;
    Hk.dd[ 0] = h11*2,  Hk.dd[ 1] = h12*2,  Hk.dd[ 2] = -h22*2,  Hk.dd[ 3] = -h21*2;
    Hk.dd[ 7] = h21*2,  Hk.dd[ 8] = h22*2,  Hk.dd[ 9] =  h12*2,  Hk.dd[10] =  h11*2;
/*	CVect3 magH = Cnb*mag;
	double C11=Cnb.e11, C01=Cnb.e01, CC=C11*C11+C01*C01;
	if(normXY(magH)>0.01 && CC>0.25)  // CC>0.25 <=> pitch<60deg
	{
		double f2=2.0/CC;
        Hk.dd[14] = (q3*C11+q0*C01)*f2,  Hk.dd[15] = (-q2*C11-q1*C01)*f2,  Hk.dd[16] = (-q1*C11+q2*C01)*f2,  Hk.dd[17] = (q0*C11-q3*C01)*f2;
		Zk.dd[2] = atan2(magH.i, magH.j);
	}
	else
	{
        Hk.dd[14] = Hk.dd[15] = Hk.dd[16] = Hk.dd[17] = 0.0;
		Zk.dd[2] = 0.0;
	}*/

	SetMeasFlag(0x03);
	TimeUpdate(ts);
	MeasUpdate();
	XPConstrain();
	normlize((CQuat*)&Xk.dd[0]);
	Cnb = q2mat(*(CQuat*)&Xk.dd[0]);
}

//*********************  class CVGHook  ************************/
CVGHook::CVGHook(void)
{
	Init(2, 600, 10);
}

void CVGHook::Init(double tau0, double maxGyro0, double maxAcc0)
{
	fasttau = 0.1; tau = tau0; maxGyro = maxGyro0; maxAcc = maxAcc0;  dkg = dka = 1.0;
 	pkf = NULL;  kfidx = 0;  overwbt = 0.0;  maxOverwbt = 10.0;
	vn = O31;
	RVG = CRAvar(3,1); 
	RVG.set(One31*1, One31*1, One31*100, One31*0.01);
}

void CVGHook::SetHook(CSINSGNSS *kf, int idx)
{
	pkf = kf;  kfidx = idx;  isEnable = 1;
	tk = pkf->kftk;
	mhny = CMahony(tau, pkf->sins.qnb);
}

void CVGHook::Enable(BOOL enable)
{
	isEnable = enable;
}

int CVGHook::Update(CVect3 &wm, CVect3 &vm, double ts)
{
	if(fasttau<tau) { fasttau+=ts; mhny.SetTau(fasttau); }
	mhny.Update(wm, vm, ts);  tk += ts;
	mhny.qnb.SetYaw(pkf->sins.att.k);
	CVect3 dan = mhny.Cnb*(vm-pkf->sins.db*ts); dan.k += pkf->sins.eth.gn.k*ts;
	vn = (1-ts/10)*vn + dan;
	RVG.Update(pow(dan*5/ts,3), ts);

	double meats = pkf->sins.nts*pkf->maxStep/pkf->tdStep;
	*(CVect3*)&pkf->Zk.dd[kfidx] = pkf->sins.vn;
	*(CVect3*)&pkf->Rt.dd[kfidx] = *(CVect3*)&RVG.R0[0];
//	*(CVect3*)&pkf->Rt.dd[kfidx] = *(CVect3*)&RVG.R0[0] *normXY(vn);
	*(CVect3*)&pkf->rts.dd[kfidx] = meats;
	if(isEnable)
		pkf->SetMeasFlag(07<<kfidx);

	if(normInf(wm)>maxGyro*DPS*ts || normInf(dan)>maxAcc*G0*ts) overwbt = maxOverwbt;
	if(overwbt>0.0)
	{
		overwbt -= ts;
		pkf->Pk.SubAddMat3(0,0, pkf->sins.Cnb*diag(pow(pkf->sins.wib*pkf->sins.nts*dkg))*pkf->sins.Cbn);
		pkf->Pk.SubAddMat3(3,3, pkf->sins.Cnb*diag(pow(pkf->sins.an *pkf->sins.nts*dka))*pkf->sins.Cbn);
		if(overwbt>9.5)	{
			*(CVect3*)&pkf->rts.dd[kfidx] = meats/100;
			*(CVect3*)&RVG.R0[0] = *(CVect3*)&RVG.Rmax[0];
		}
		else {
			*(CVect3*)&pkf->rts.dd[kfidx] = meats*10000;
			*(CVect3*)&RVG.R0[0] = *(CVect3*)&RVG.Rmin[0];
		}
		return 2;
	}

	if(pkf->sins.an.k>-2*(G0+0.5) && pkf->sins.an.k<-2*(G0-0.5) && RVG.R0[2]<0.1) {
		pkf->sins.qnb = UpDown(pkf->sins.qnb);  pkf->sins.vn.k = 0;
		return 3;
	}
	return 1;
}

#endif  // PSINS_AHRS_MEMS

//******************************  File Read or Write *********************************/
#ifdef PSINS_IO_FILE
#pragma message("  PSINS_IO_FILE")

char* timestr(int type, char *p)
{
	static char PSINStimestr[16];
	if(p==NULL) p=PSINStimestr;
	time_t tt;  time(&tt);
	tm *Time = localtime(&tt);
	if(type==0)		 strftime(p, 32, "%Y%m%d_%H%M%S", Time); // YYYYmmdd_HHMMSS
	else if(type==1) strftime(p, 32, "%Y%m%d", Time);		 // YYYYmmdd
	else if(type==2) strftime(p, 32, "%H%M%S", Time);		 // HHMMSS
	return p;
}

char* time2fname(void)
{
	static char PSINSfname[32]="PSINSYYYYmmdd_HHMMSS.bin";
	timestr(0, &PSINSfname[5]);  PSINSfname[20] = '.';
	return PSINSfname;
}

char CFileRdWt::dirIn[256] = {0}, CFileRdWt::dirOut[256] = {0};

void CFileRdWt::DirI(const char *dirI)  // set dirIN
{
	int len = strlen(dirI);
	memcpy(dirIn, dirI, len);
	if(dirIn[len-1]!='\\') { dirIn[len]='\\'; dirIn[len+1]='\0'; }
	if(access(dirIn,0)==-1) dirIn[0]='\0';
}

void CFileRdWt::DirO(const char *dirO)  // set dirOUT
{
	int len = strlen(dirO);
	memcpy(dirOut, dirO, len);
	if(dirOut[len-1]!='\\') { dirOut[len]='\\'; dirOut[len+1]='\0'; }
	if(access(dirOut,0)==-1) dirOut[0]='\0';
}

void CFileRdWt::Dir(const char *dirI, const char *dirO)  // set dirIN&OUT
{
	DirI(dirI);
	DirO(dirO?dirO:dirI);
}

CFileRdWt::CFileRdWt()
{
	rwf = 0;
}

CFileRdWt::CFileRdWt(const char *fname0, int columns0)
{
	rwf = 0;
	Init(fname0, columns0);
	memset(buff, 0, sizeof(buff));
}

CFileRdWt::~CFileRdWt()
{
	if(rwf) { fclose(rwf); rwf=(FILE*)NULL; } 
}

void CFileRdWt::Init(const char *fname0, int columns0)
{
	fname[0]='\0';
	int findc=0, len0=strlen(fname0);
	for(int i=0; i<len0; i++)	{ if(fname0[i]=='\\') { findc=1; break; } }
	columns = columns0;
	if(columns==0)		// file write
	{	if(dirOut[0]!=0&&findc==0)	{ strcat(fname, dirOut); } }
	else				// file read
	{	if(dirIn[0]!=0&&findc==0)	{ strcat(fname, dirIn); } }
	strcat(fname, fname0);
	if(rwf) this->~CFileRdWt();
	if(columns==0)				// bin file write
	{
		rwf = fopen(fname, "wb");
	}
	else if(columns<0)			// bin file read
	{
		rwf = fopen(fname, "rb");
	}
	else if(columns>0)			// txt file read
	{
		rwf = fopen(fname, "rt");
		if(!rwf) return;
		long pos;
		while(1)  // skip txt-file comments
		{
			pos = ftell(rwf);
			fgets(line, sizeof(line), rwf);
			if(feof(rwf)) break;
			int allSpace=1, allDigital=1;
			for(int i=0; i<sizeof(line); i++)
			{
				char c = line[i];
				if(c=='\n') break;
				if(c!=' ') allSpace = 0;
				if( !(c==' '||c==','||c==';'||c==':'||c=='+'||c=='-'||c=='.'||c=='\t'
					||c=='e'||c=='E'||c=='d'||c=='D'||('9'>=c&&c>='0')) )
					allDigital = 0;
			}
			if(!allSpace && allDigital) break;
		}
		fseek(rwf, pos, SEEK_SET);
//		this->columns = columns;
		for(int i=0; i<columns; i++)
		{ sstr[4*i+0]='%', sstr[4*i+1]='l', sstr[4*i+2]='f', sstr[4*i+3]=' ', sstr[4*i+4]='\0'; } 
	}
	else
	{
		rwf = 0;
	}
	linek = 0;
	totsize = 0;
	svpos[0]=svpos[1]=svpos[2]=ftell(rwf);  items[0]=items[1]=items[2]=0;
}

int CFileRdWt::load(int lines, BOOL txtDelComma)
{
	if(columns<0)			// bin file read
	{
		if(lines>1)	fseek(rwf, (lines-1)*(-columns)*sizeof(double), SEEK_CUR);
		fread(buff, -columns, sizeof(double), rwf);
		if(lines==0) fseek(rwf, columns*sizeof(double), SEEK_CUR);
	}
	else					// txt file read
	{
		for(int i=0; i<lines; i++)  fgets(line, sizeof(line), rwf);
		if(txtDelComma)
		{
			for(char *pc=line, *pend=line+sizeof(line); pc<pend; pc++)
			{
				if(*pc==','||*pc==';'||*pc==':'||*pc=='\t') *pc=' ';
				else if(*pc=='\0') break;
			}
		}
		if(columns<=10)
			sscanf(line, sstr,
				&buff[ 0], &buff[ 1], &buff[ 2], &buff[ 3], &buff[ 4], &buff[ 5], &buff[ 6], &buff[ 7], &buff[ 8], &buff[ 9]
				); 
		else if(columns<=20)
			sscanf(line, sstr,
				&buff[ 0], &buff[ 1], &buff[ 2], &buff[ 3], &buff[ 4], &buff[ 5], &buff[ 6], &buff[ 7], &buff[ 8], &buff[ 9],
				&buff[10], &buff[11], &buff[12], &buff[13], &buff[14], &buff[15], &buff[16], &buff[17], &buff[18], &buff[19]
				); 
		else if(columns<=40)
			sscanf(line, sstr,
				&buff[ 0], &buff[ 1], &buff[ 2], &buff[ 3], &buff[ 4], &buff[ 5], &buff[ 6], &buff[ 7], &buff[ 8], &buff[ 9],
				&buff[10], &buff[11], &buff[12], &buff[13], &buff[14], &buff[15], &buff[16], &buff[17], &buff[18], &buff[19],
				&buff[20], &buff[21], &buff[22], &buff[23], &buff[24], &buff[25], &buff[26], &buff[27], &buff[28], &buff[29],
				&buff[30], &buff[31], &buff[32], &buff[33], &buff[34], &buff[35], &buff[36], &buff[37], &buff[38], &buff[39]
				); 
	}
	linek += lines;
	if(feof(rwf))  return 0;
	else return 1;
}

int CFileRdWt::loadf32(int lines)	// float32 bin file read
{
	if(lines>1)
		fseek(rwf, (lines-1)*(-columns)*sizeof(float), SEEK_CUR);
	fread(buff32, -columns, sizeof(float), rwf);
	for(int i=0; i<-columns; i++) buff[i]=buff32[i];	// float->double copy
	linek += lines;
	if(feof(rwf))  return 0;
	else return 1;
}

long CFileRdWt::load(BYTE *buf, long bufsize)  // load bin file
{
	long cur = ftell(rwf);
	fseek(rwf, 0L, SEEK_END);
	long rest = ftell(rwf)-cur;
	fseek(rwf, cur, SEEK_SET);
	if(bufsize>rest) bufsize=rest;
	fread(buf, bufsize, 1, rwf);
	return bufsize;
}

void CFileRdWt::bwseek(int lines, int mod)  // double bin-file backward-seek lines
{
	fseek(rwf, lines*(-columns)*sizeof(double), mod);
	linek -= lines;
}

long CFileRdWt::filesize(int opt)
{
	long cur = ftell(rwf);
	if(totsize==0)
	{
		fseek(rwf, 0L, SEEK_END);
		totsize = ftell(rwf);			// get total_size
		fseek(rwf, cur, SEEK_SET);
	}
	remsize = totsize-cur;
	return opt ? remsize : totsize;  // opt==1 for remain_size, ==0 for total_size
}

int CFileRdWt::getl(void)	// txt file get a line
{
	fgets(line, sizeof(line), rwf);
	return strlen(line);
}

long CFileRdWt::savepos(int i)
{
	if(i<0||i>2) i=0;
	items[i] = linek;
	return (svpos[i]=ftell(rwf));
}

int CFileRdWt::restorepos(int i)
{
	if(i<0||i>2) i=0;
	linek = items[i];
	return fseek(rwf, svpos[i], SEEK_SET);
}

BOOL CFileRdWt::waitfor(int columnk, double val, double eps)
{
	BOOL is32=0;
	if(columnk>=100) { is32=1; columnk%=100; }  // columnk>=100 for load float data, using loadf32()
	double wf=buff[columnk]-val;
	while(-eps<wf && wf<eps) {
		is32 ? loadf32(1) : load(1);
		if(feof(rwf)) return 0;
		wf = buff[columnk] - val;
	}
	return 1;
}

CFileRdWt& CFileRdWt::operator<<(double d)
{
	fwrite(&d, 1, sizeof(double), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator<<(const CVect3 &v)
{
	fwrite(&v, 1, sizeof(v), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator<<(const CQuat &q)
{
	fwrite(&q, 1, sizeof(q), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator<<(const CMat3 &m)
{
	fwrite(&m, 1, sizeof(m), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator<<(const CVect &v)
{
	fwrite(v.dd, v.clm*v.row, sizeof(double), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator<<(const CMat &m)
{
	fwrite(m.dd, m.clm*m.row, sizeof(double), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator<<(const CRAvar &R)
{
	fwrite(R.R0, R.nR0, sizeof(double), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator<<(const CMaxMinn &mm)
{
	for(int i=0; i<mm.n; i++)
		*this<<mm.mm[i].maxRes<<mm.mm[i].minRes<<mm.mm[i].maxpreRes<<mm.mm[i].minpreRes<<mm.mm[i].meanRes<<mm.mm[i].diffRes<<(double)mm.mm[i].flag;
	return *this;
}

CFileRdWt& CFileRdWt::operator<<(const CAlignsb &aln)
{
	return *this<<aln.att<<aln.eb<<aln.db<<aln.tk;
}

CFileRdWt& CFileRdWt::operator<<(const CAligni0 &aln)
{
	return *this<<q2att(aln.qnb)<<aln.vib0<<aln.Pi02<<aln.tk;
}

CFileRdWt& CFileRdWt::operator<<(const CPolyfit &pfit)
{
	for(int i=0; i<pfit.n; i++) *this<<pfit.Xk[i];
	return *this;
}

CFileRdWt& CFileRdWt::operator<<(const CPolyfit3 &pfit)
{
	for(int i=0; i<pfit.n; i++) *this<<pfit.Xkv[i];
	return *this;
}

CFileRdWt& CFileRdWt::operator<<(const CIMU &imu)
{
	return *this<<imu.phim<<imu.dvbm<<imu.tk;
}

CFileRdWt& CFileRdWt::operator<<(const CSINS &sins)
{
	if(sins.isOutlever)
		return *this<<sins.att<<sins.vnL<<sins.posL<<sins.eb<<sins.db<<sins.tk;
	else
		return *this<<sins.att<<sins.vn<<sins.pos<<sins.eb<<sins.db<<sins.tk;
}

CFileRdWt& CFileRdWt::operator<<(const CDR &dr)
{
	return *this<<dr.att<<dr.vn<<dr.pos<<dr.dist<<dr.tk;
}

#ifdef PSINS_RMEMORY
CFileRdWt& CFileRdWt::operator<<(const CRMemory &m)
{
	fwrite(m.pMemStart0, m.memLen, 1, rwf);
	return *this;
}
#endif

#ifdef PSINS_AHRS_MEMS
CFileRdWt& CFileRdWt::operator<<(const CMahony &ahrs)
{
	return *this<<m2att(ahrs.Cnb)<<ahrs.exyzInt<<ahrs.tk;
}

CFileRdWt& CFileRdWt::operator<<(const CQEAHRS &ahrs)
{
	return *this<<m2att(ahrs.Cnb)<<*(CVect3*)&ahrs.Xk.dd[4]<<diag(ahrs.Pk)<<ahrs.kftk;
}

CFileRdWt& CFileRdWt::operator<<(const CVGHook &vg)
{
	return *this<<m2att(vg.mhny.Cnb)<<(~vg.mhny.Cnb)*vg.vn<<vg.RVG<<vg.tk;
}
#endif

#ifdef PSINS_CONSOLE_UART
CFileRdWt& CFileRdWt::operator<<(const CUartPP &uart)
{
	fwrite(uart.popbuf, uart.frameLen, sizeof(char), rwf);
	return *this;
}
#endif

CFileRdWt& CFileRdWt::operator<<(CKalman &kf)
{
	*this<<kf.Xk<<diag(kf.Pk)<<kf.Zk<<kf.Rt<<(double)kf.measflaglog<<kf.kftk;
	kf.measflaglog = 0;
	return *this;
}

CFileRdWt& CFileRdWt::operator>>(double &d)
{
	fread(&d, 1, sizeof(double), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator>>(CVect3 &v)
{
	fread(&v, 1, sizeof(v), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator>>(CQuat &q)
{
	fread(&q, 1, sizeof(q), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator>>(CMat3 &m)
{
	fread(&m, 1, sizeof(m), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator>>(CVect &v)
{
	fread(v.dd, v.clm*v.row, sizeof(double), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator>>(CMat &m)
{
	fread(m.dd, m.clm*m.row, sizeof(double), rwf);
	return *this;
}

//******************************  CFImuGpsSync *********************************/	
CFImuGnssSync::CFImuGnssSync(const char *imu, int imuLen, double ts, const char *gps, int gpsLen)
{
	tgps = &gpsBuf[gpsLen-1];	tgpsNext = &gpsBufNext[gpsLen-1];	res = 0;
	this->imuLen = imuLen, this->ts = ts, this->gpsLen = gpsLen;  timu = 0.0;
	char fname1[256]={0};  if(CFileRdWt::dirIn[0]!=0) strcat(fname1, CFileRdWt::dirIn);  strcat(fname1, imu);
	fimu = fopen(fname1, "rb");
	char fname2[256]={0};  if(CFileRdWt::dirIn[0]!=0) strcat(fname2, CFileRdWt::dirIn);  strcat(fname2, gps);
	fgps = fopen(fname2, "rb");
	fread(gpsBuf, gpsLen*sizeof(double), 1, fgps);
	fread(gpsBufNext, gpsLen*sizeof(double), 1, fgps);
	load(1);
}

CFImuGnssSync::~CFImuGnssSync()
{
	if(fimu) fclose(fimu); fimu=NULL;
	if(fgps) fclose(fgps); fgps=NULL;
}

int CFImuGnssSync::load(int i)
{
	res = 1;  // IMU OK
	if(i>1) fseek(fimu, (i-1)*imuLen*sizeof(float), SEEK_CUR);
	fread(imuBuf32, imuLen*sizeof(float), 1, fimu);
	if(feof(fimu))  return res=0;  // no IMU
	float *pf32=imuBuf32;
	for(double *pf=imuBuf, *pEnd=&imuBuf[imuLen]; pf<pEnd; pf++,pf32++) *pf=*pf32; // float->double copy
	if(ts>0.0) timu+=i*ts; else timu=imuBuf[imuLen-1];  // if ts<=0, then imuBuf[imuLen-1] is time tag
	while(timu>=*tgpsNext) {
		memcpy(gpsBuf, gpsBufNext, gpsLen*sizeof(double));
		fread(gpsBufNext, gpsLen*sizeof(double), 1, fgps);
		if(feof(fgps)) return res=-1;  // no GPS
		res = 2;  // IMU & GPS OK
	}
	dT = *tgps-timu;
	return res;
}

#ifdef PSINS_IO_FILE_FIND

//******************************  CFileList *********************************/	
CFileList::CFileList(const char *listf, const char *listout, const char *firstf, const char *lastf)
{
	handle = iNextFile = 0;  flistout = rwf = NULL;
	fname[0]='\0';
	strcat(strcat(fname, dirIn), listf);
	for(unsigned int i=0; i<strlen(listf); i++) {
		if(listf[i]=='*') {					// using asterisk wildcard to find files
			handle = _findfirst(fname, &file);
			if(firstf&&lastf==NULL) { lastf=firstf; firstf=listout; listout="listout.txt"; } // CFileList(const char *listf, const char *firstf, const char *lastf)
			int res=0; 
			if(firstf) { 
				while(res==0&&strcmp(file.name, firstf)) res=_findnext(handle, &file);
			}
			if(res==-1) { _findclose(handle); handle=-1; }
			else		{ this->lastf = lastf; isLastf = 0; }
			if(handle==-1) printf("No file found!\n");
			break;
		}
	}
	if(!handle) rwf = fopen(fname, "rt");	// using input txt-file to find files
	if(listout) {							// output the founded files to a txt-file
		fname[0]='\0';
		strcat(strcat(fname, dirIn), listout);
		flistout = fopen(fname, "wt");
	}
	iSubfix = 0;
}

char* CFileList::NextFile(void)
{
	if(handle) {	// using asterisk wildcard to find files
		if(handle==-1) return NULL;
		int res=0;
		if(iNextFile>0) res=_findnext(handle, &file);
		if(res==0) strcpy(fname, file.name);
		else return NULL;
		if(lastf&&strcmp(fname, lastf)==0) { lastf=NULL; isLastf=1; }
		else if(isLastf) return NULL;
	}
	else if(rwf) {	// using input txt-file to find files
		int len;
		while(1) {
			len = getl();
			if(len==0||feof(rwf)||line[0]=='#') return NULL;
			if(line[0]=='%'||line[0]=='/') continue;
			break;
		}
		for(int i=len; i>0; i--) if(line[i]==' ') { line[i]='\0'; break; }
		strcpy(fname, line);
	}
	iNextFile++;
	printf("--- %s\t%d\n", fname, iNextFile);
	if(flistout) fprintf(flistout, "%s\t %d\n", fname, iNextFile); // output the founded files to a txt-file
	char subfix[32]; if(iSubfix) sprintf(subfix, "%s", &fname[strlen(fname)-iSubfix-4]); else sprintf(subfix, "%03d.bin", iNextFile);
	sprintf(fins, "ins%s", subfix);  sprintf(fkff, "kff%s", subfix); // create some useful file names
	sprintf(fxxx, "xxx%s", subfix);  sprintf(fyyy, "yyy%s", subfix);  sprintf(fzzz, "zzz%s", subfix);
	return fname;
}

CFileList::~CFileList()
{
	if(handle>0) { _findclose(handle); handle=0; }
	if(flistout) { fclose(flistout); flistout=0; }
	CFileRdWt::~CFileRdWt();
}

#endif // PSINS_IO_FILE_FIND

//******************************  CFileLog *********************************/	
CFileLog::CFileLog(void)
{
	f = f0 = (FILE*)NULL;  nLeft=0; CorMArray=1;
	tms0 = clock();
}

CFileLog& CFileLog::LogSet(BOOL isLog, const char *fname)
{
	if(isLog==0) {		// stop log
		f=(FILE*)NULL;
	}
	else if(isLog==1) {	// start/re-start log with file name 'psinslog.txt' or '*fname'
		f=f0;
		if(!f) { f=fopen(fname, "at+"); f0=f; }
	}
	else if(isLog==2||isLog==3) {	// start/re-start log with file name 'psinslogYYmmdd.txt'/'psinslogYYmmdd_HHMMSS.txt'
		f=f0;
		if(!f) {
			char str[64];
			time_t tt;  time(&tt);
			tm *Time = localtime(&tt);
			strftime(str, 32, isLog==2?"psinslog%Y%m%d.txt":"psinslog%Y%m%d_%H%M%S.txt", Time);
			f=fopen(str, "at+"); f0=f;
		}
	}
	return LogDate();
}

CFileLog::~CFileLog()
{
	psinslog.LogRunTime();
	if(f) { fclose(f); f=f0=(FILE*)NULL; } 
}

CFileLog& CFileLog::LogDate(BOOL hmsOnly)
{
	char str[64], *str1=str;
	time_t tt;  time(&tt);
	tm *Time = localtime(&tt);
	strftime(str, 32, "\n---- %Y/%m/%d %H:%M:%S ----\n", Time);
	if(hmsOnly) { str1=&str[17]; str1[9]='\0'; }
	return *this<<str1;
}

CFileLog& CFileLog::LogRunTime(BOOL abst)
{
	clock_t tms1 = clock();
	if(f) { fprintf(f, "Elasped time (ms): %ld.\n", tms1-tms0); Flush(1); }
	if(abst==0) tms0 = tms1;
	return *this;
}

CFileLog& CFileLog::operator<<(const char *str)
{
	if(f) { fprintf(f, "%s", str); Flush(1); }
	return *this;
}

CFileLog& CFileLog::operator<<(int i)
{
	if(f) { fprintf(f, "\t%d ", i); Flush(1); }
	return *this;
}

CFileLog& CFileLog::operator<<(float ff)
{
	if(f) { fprintf(f, "\t%f ", ff); Flush(1); }
	return *this;
}

CFileLog& CFileLog::operator<<(double d)
{
	if(f) { fprintf(f, "\t%.8e ", d); Flush(1); }
	return *this;
}

CFileLog& CFileLog::operator<<(const CVect3 &v)
{
	if(f) { fprintf(f, "\t%.8e\t%.8e\t%.8e ", v.i, v.j, v.k); Flush(3); }
	return *this;
}

CFileLog& CFileLog::operator<<(const CQuat &q)
{
	if(f) { fprintf(f, "\t%.8e\t%.8e\t%.8e\t%.8e ", q.q0, q.q1, q.q2, q.q3); Flush(4); }
	return *this;
}

CFileLog& CFileLog::operator<<(const CVect &v)
{
	if(f) {
		for(int i=0; i<v.rc; i++) fprintf(f, "\t%.8e", v.dd[i]);
		Flush(3);
	}
	return *this;
}

CFileLog& CFileLog::operator<<(const CMat3 &m)
{
	if(f) {
		fprintf(f, "\t%.8e\t%.8e\t%.8e\n", m.e00, m.e01, m.e02);
		fprintf(f, "\t%.8e\t%.8e\t%.8e\n", m.e10, m.e11, m.e12);
		fprintf(f, "\t%.8e\t%.8e\t%.8e ", m.e20, m.e21, m.e22);
		Flush(9);
	}
	return *this;
}

CFileLog& CFileLog::operator<<(const CMat &m)
{
	if(f) {
		const double *p=m.dd; 
		for(int i=0; i<m.row; i++) {
			for(int j=0; j<m.clm; j++)
				fprintf(f, "\t%.8e", *p++);
			fprintf(f, "\n");
		}
		Flush(m.rc);
	}
	return *this;
}

CFileLog& CFileLog::CMArray(const char* varname, const double *var, int row, int clm, double scale, const char* comment)
{
	return CorMArray==1 ? CArray(varname, var, row, clm, scale, comment) : MArray(varname, var, row, clm, scale, comment);
}

CFileLog& CFileLog::CArray(const char* varname, const double *var, int row, int clm, double scale, const char* comment)
{
	if(f) {
		if(clm==1) { clm=row;  row=1; }  // CArray(varname, var, clm)
		if(row==1) fprintf(f, "double %s[%d] = {", varname, clm);
		else       fprintf(f, "double %s[%d][%d] = {", varname, row, clm);
		if(comment) fprintf(f, "\t// %s\n", comment); else fprintf(f, "\n");
		for(int j=0; j<row; j++) {
			for(int i=0; i<clm; i++) {
				if(i==0)  { if(row==1) fprintf(f, "\t%12.8e,", scale**var++); else fprintf(f, "\t{%12.8e,", scale**var++); }
				else if(i==clm-1)  { if(row==1) fprintf(f, "\t%12.8e,\n", scale**var++); else fprintf(f, "\t%12.8e},\n", scale**var++); }
				else fprintf(f, "\t%12.8e,", scale**var++);
			}
		}
		if(row==1&&clm==1) fprintf(f, "\n");
		fprintf(f, "\t};\n");
	}
	return *this;
}

CFileLog& CFileLog::MArray(const char* varname, const double *var, int row, int clm, double scale, const char* comment)
{
	if(f) {
		int prime=0;
		if(clm==1) { clm=row;  row=1; prime=1; }
		fprintf(f, "%s = [", varname, row, clm);
		if(comment) fprintf(f, "\t%% %s\n", comment); else fprintf(f, "\n");
		for(int j=0; j<row; j++) {
			for(int i=0; i<clm; i++) {
				if(i==clm-1)  fprintf(f, "\t%12.8e,\n", scale**var++);
				else fprintf(f, "\t%12.8e,", scale**var++);
			}
		}
		if(prime) fprintf(f, "\t]';\n");	else fprintf(f, "\t];\n");
	}
	return *this;
}
	
void CFileLog::Flush(int n)
{
	if(f && (nLeft+=n)>100) { fflush(f);  nLeft=0; }
}

//******************************  CFileCfg *********************************/
CFileCfg::CFileCfg(void)
{
	f = NULL;  isHdr = 0;
}

CFileCfg::~CFileCfg()
{
	if(isHdr && f) { fclose(f); f = NULL; }
}

CFileCfg& CFileCfg::operator<<(const char *hdr)
{
	if(f) {
		char hdr0[CfgHdrLen];
		memset(hdr0, 0, CfgHdrLen);
		memcpy(hdr0, hdr, mmin(CfgHdrLen, strlen(hdr)));
		fwrite(hdr0, 1, CfgHdrLen, f);  isHdr=1;
	}
	return *this;
}

CFileCfg& CFileCfg::operator>>(const char *hdr)
{
	if(f) {
		char hdr0[CfgHdrLen];
		fread(hdr0, 1, CfgHdrLen, f);  isHdr=2;
		int len = mmin(CfgHdrLen,strlen(hdr));
		for(int i=0; i<len; i++) { if(hdr0[i]!=hdr[i]) { isHdr=-1; break; } }
	}
	return *this;
}

CFileCfg& CFileCfg::operator<<(short s)
{
	fwrite(&s, 1, sizeof(s), f);
	return *this;
}

CFileCfg& CFileCfg::operator<<(int i)
{
	fwrite(&i, 1, sizeof(i), f);
	return *this;
}

CFileCfg& CFileCfg::operator<<(float ff)
{
	fwrite(&ff, 1, sizeof(ff), f);
	return *this;
}

CFileCfg& CFileCfg::operator<<(double d)
{
	fwrite(&d, 1, sizeof(d), f);
	return *this;
}

CFileCfg& CFileCfg::operator<<(const CVect3 &v)
{
	fwrite(&v, 1, sizeof(v), f);
	return *this;
}

CFileCfg& CFileCfg::operator<<(const CQuat &q)
{
	fwrite(&q, 1, sizeof(q), f);
	return *this;
}

CFileCfg& CFileCfg::operator<<(const CMat3 &m)
{
	fwrite(&m, 1, sizeof(m), f);
	return *this;
}

CFileCfg& CFileCfg::operator<<(const CMat &m)
{
	fwrite(&m.dd[0], 1, sizeof(double)*m.rc, f);
	return *this;
}

CFileCfg& CFileCfg::operator>>(short &s)
{
	isHdr==2 ? fread(&s, 1, sizeof(s), f) : s=0;
	return *this;
}

CFileCfg& CFileCfg::operator>>(int &i)
{
	isHdr==2 ? fread(&i, 1, sizeof(i), f) : i=0;
	return *this;
}

CFileCfg& CFileCfg::operator>>(float &ff)
{
	isHdr==2 ? fread(&ff, 1, sizeof(ff), f) : ff=0.0;
	return *this;
}

CFileCfg& CFileCfg::operator>>(double &d)
{
	isHdr==2 ? fread(&d, 1, sizeof(d), f) : d=0.0;
	return *this;
}

CFileCfg& CFileCfg::operator>>(CVect3 &v)
{
	isHdr==2 ? fread(&v, 1, sizeof(v), f) : v=O31;
	return *this;
}

CFileCfg& CFileCfg::operator>>(CQuat &q)
{
	isHdr==2 ? fread(&q, 1, sizeof(q), f) : q=qI;
	return *this;
}

CFileCfg& CFileCfg::operator>>(CMat3 &m)
{
	isHdr==2 ? fread(&m, 1, sizeof(m), f) : m=I33;
	return *this;
}

CFileCfg& CFileCfg::operator=(CFileCfg &cfg)
{
	f = NULL;		// =NULL, assignment operator forbidden ?
	return *this;
}

CFileCfg WriteCfg(const char *fname, const char *ext)  // WriteCfg("cfg.bin")<<"kked"<<Kg<<eb<<Ka<<db;
{
	char name[256];
	if(ext) fname=strcat(strcpy(name,fname),ext);
	CFileCfg cfg;
	cfg.f = fopen(fname, "wb");
	return cfg;
}

CFileCfg ReadCfg(const char *fname, const char *ext)  // { ReadCfg("cfg.bin")>>"kked">>Kg>>eb>>Ka>>db; }
{
	char name[256];
	if(ext) fname=strcat(strcpy(name,fname),ext);
	CFileCfg cfg;
	cfg.f = fopen(fname, "rb");
	return cfg;
}

#endif // PSINS_IO_FILE

//******************************  CRMemory *********************************/
#ifdef PSINS_RMEMORY
#pragma message("  PSINS_RMEMORY")

CRMemory::CRMemory(void)
{
	pMemStart0 = NULL;
}

CRMemory::CRMemory(long recordNum0, int recordLen0)
{
	pMemStart0 = NULL;
	recordNum = recordNum0;
	BYTE *pb = (BYTE*)malloc(recordNum0*recordLen0);
	Init(pb, recordNum0*recordLen0, recordLen0);
	pMemStart0 = pMemStart;
}

CRMemory::CRMemory(BYTE *pMem, long memLen0, int recordLen0)
{
	Init(pMem, memLen0, recordLen0);
}

void CRMemory::Init(BYTE *pMem, long memLen0, int recordLen0)
{
	psinsassert(recordLen0<=MAX_RECORD_BYTES);
	pMemStart0 = pMemStart = pMemPush = pMemPop = pMem;
	pMemEnd = pMemStart + memLen0;
	recordNum = memLen0/recordLen0;
	pushLen = popLen = recordLen = recordLen0;
	memLen = memLen0;
	dataLen = 0;
}

CRMemory::~CRMemory()
{
	if(pMemStart0) { free(pMemStart0); pMemStart0 = NULL; } 
}

void CRMemory::operator=(const CRMemory &m)
{
	BYTE *pb=this->pMemStart0;
	int len=recordNum*recordLen, len0=m.recordNum*m.recordLen;
	memcpy(this, &m, sizeof(m));
	this->pMemStart0 = pb;
	if(len!=len0) {
		Deletep(this->pMemStart0);
		BYTE *pMem = (BYTE*)malloc(len0);
		Init(pMem, len0, m.recordLen);
	}
	this->pMemStart=this->pMemStart0+(m.pMemStart-m.pMemStart0);
	this->pMemEnd=this->pMemStart0+(m.pMemEnd-m.pMemStart0);
	this->pMemPush=this->pMemStart0+(m.pMemPush-m.pMemStart0);
	this->pMemPop=this->pMemStart0+(m.pMemPop-m.pMemStart0);
	memcpy(this->pMemStart0, m.pMemStart0, len0);
}

BYTE CRMemory::pop(BYTE *p)
{
	if(dataLen==0) return 0;
	popLen = recordLen==0 ? *pMemPop : recordLen;
	if(p==(BYTE*)NULL) p = popBuf;
	int i;
	for(i=0; i<popLen; i++,dataLen--)
	{
		*p++ = *pMemPop++;
		if(pMemPop>=pMemEnd)  pMemPop = pMemStart;
	}
	return i;
}

BYTE* CRMemory::get(int iframe)
{
	iframe = iframe % recordNum;
	if(iframe<0) iframe += recordNum;
	return &pMemStart[popLen*iframe];
}

BYTE* CRMemory::set(int iframe, const BYTE *p)
{
	BYTE *p0 = &pMemStart[pushLen*iframe];
	for(int i=0; i<pushLen; i++)
	{
		*p0++ = *p++;
		if(p0>=pMemEnd)  p0 = pMemStart;
	}
	return &pMemStart[pushLen*iframe];
}

BOOL CRMemory::push(const BYTE *p)
{
	BOOL res = 1;
	if(p==(BYTE*)NULL) p = pushBuf;
	pushLen = recordLen==0 ? *p : recordLen;
	psinsassert(pushLen<=MAX_RECORD_BYTES);
	for(int i=0; i<pushLen; i++,dataLen++)
	{
		*pMemPush++ = *p++;
		if(pMemPush>=pMemEnd) pMemPush = pMemStart;
		if(pMemPush==pMemPop) { res=0; pop(); }
	}
	return res;
}

//******************************  CSmooth *********************************/
CSmooth::CSmooth(int clm, int row)
{
	psinsassert(clm<=MMD);
	irow = 0; maxrow = row;
	pmem = new CRMemory(row+2, clm*sizeof(double));
	sum = mean = tmp = CVect(clm, 0.0);
}

CSmooth::~CSmooth()
{
	Deletep(pmem);
}

CVect CSmooth::Update(const double *p, double *pmean)
{
	pmem->push((unsigned char*)p);
	sum += CVect(tmp.rc,p);
	if(irow<maxrow) {
		irow++;
		tmp = 0.0;
	}
	else {
		pmem->pop((unsigned char*)tmp.dd);
	}
	sum -= tmp;
	mean = sum*(1.0/irow);
	if(pmean) memcpy(pmean, mean.dd, mean.rc*sizeof(double));
	return mean;
}

//******************************  CInterp *********************************/
CInterp::CInterp(double **table, int row, int clm)
{
	this->row = row; this->clm = clm;
	mem = CRMemory((BYTE*)table, row*clm*sizeof(double), clm*sizeof(double));
	pmem = &mem;
	mint = table[0][0];  maxt = table[row-1][0];  irow = 0;
}

CInterp::CInterp(const char *fname, int clm)
{
	this->clm = clm;
	CFileRdWt f(fname, -clm);
	long sz = f.filesize();  this->row = sz/(clm*sizeof(double))+2;
	pmem = new CRMemory(row, clm*sizeof(double));
	fread(pmem->get(1), sz, 1, f.rwf);
	double *p = (double*)pmem->set(0, pmem->get(1)); *p=mint=-INF;
	p = (double*)pmem->set(row-1, pmem->get(row-2)); *p=maxt=INF;
	irow = 0;
}

CInterp::~CInterp()
{
	if(pmem->pMemStart0) Deletep(pmem);
}

double CInterp::Interp(double t, double *data)
{
	double *p0, *p1;
	p0 = (double*)pmem->get(irow);
	int OK_t=0;
	if(p0[0]<t) {
		p1 = p0+clm;
		if(t<=p1[0]) OK_t=1;     // p0[0] < t <= p1[0],  cur interval
		else if(p1[0]<t) {
			p0 = p1;
			p1 = p0+clm;  irow++;
			if(t<=p1[0]) OK_t=1;     // p1[0] < t <= p2[0], next interval
		}
	}
	else if(t<=p0[0]) {
		p1 = p0;
		p0 = p1-clm;  irow--;
		if(p0[0]<t) OK_t=1;     // p_1[0] < t <= p0[0], pre interval
	}
	if(!OK_t) {
		if(t<=mint) t=mint; else if(t>=maxt) t=maxt;
		p0 = (double*)pmem->pMemStart;
//		for(irow=0; irow<row; irow++,p0+=clm) {  // re-find from beginning
//			if(p0[0]<t && t<=p0[clm]) break;
//		}
//		if(irow>=row-1) return INF;  // no find, error
		for(int row1=0,row2=row,i=0; i<row; i++) {  // binary search
			irow = (row1+row2)/2;
			if(irow==row1) break;
			if(t<=p0[irow*clm])	row2 = irow;
			else row1 = irow;
		}
		p0 = (double*)pmem->get(irow);  p1 = p0+clm;
	}
	double k=(t-p0[0])/(p1[0]-p0[0]);
	if(data) {
		for(int i=1; i<clm; i++) {
			data[i-1] = p0[i] + (p1[i]-p0[i])*k;
		}
//data[0]=data[1]=data[2]=0.0;
		return data[0];
	}
	else
		return p0[1] + (p1[1]-p0[1])*k;
};

#endif // PSINS_RMEMORY

BYTE* flipud(BYTE *p, int rows, int clmBytes)
{
	BYTE *ptmp=(BYTE*)malloc(clmBytes), *p0=p, *p1=p0+(rows-1)*clmBytes, *p11=p1;
	memcpy(ptmp, p0, clmBytes);
	while(1) {
		memcpy(p0, p1, clmBytes);  p0+=clmBytes;
		if(p0>=p1) break;
		memcpy(p1, p0, clmBytes);  p1-=clmBytes;
		if(p0>=p1) break;
	}
	memcpy(p0, p0+clmBytes, p11-p0);
	memcpy(p11, ptmp, clmBytes);
	Deletep(ptmp);
	return p;
}

void deal32(float *pf, ...)  // deal32(buff[], &pf1,0, &pf2,3, &pf3,6, NULL);
{
	va_list vl;
	va_start(vl, pf);  
	for(int i=0; i<50; i++) {
		void **p=va_arg(vl,void**);  int idx = va_arg(vl, int);
		if(p==NULL) break;
		*p = (void*)&pf[idx];
	}
	va_end(vl);
}

void deal(double *pf, ...)  // deal(buff[], &pwm,0, &pvm,3, &pt,6, NULL);
{
	va_list vl;
	va_start(vl, pf);  
	for(int i=0; i<50; i++) {
		void **p=va_arg(vl,void**);  int idx = va_arg(vl, int);
		if(p==NULL) break;
		*p = (void*)&pf[idx];
	}
	va_end(vl);
}

//******************************  CContinuousCnt *******************************/
CContinuousCnt::CContinuousCnt(int cntLargest)
{
	cnt0=cntLargest0=cntLargest; isFirst=1; lost=0;
}

int CContinuousCnt::Update(int cnt)		// cnt = 0 ~ cntLargest0
{
	if(isFirst) { cnt0=cnt; isFirst=0; return 1; }
	int dcnt=cnt-cnt0; cnt0=cnt;
	if(dcnt<0) dcnt+=cntLargest0+1;
	lost += dcnt; lost--;
	return dcnt;
}

//******************************  CVCFileFind *********************************/
#ifdef PSINS_VC_AFX_HEADER

CVCFileFind::CVCFileFind(char *path, char *filetype)
{
	finder.FindFile(strcat(strcpy(fname, path), filetype));
	lastfile = 0;
}

char *CVCFileFind::FindNextFile(void)
{
	if(lastfile) return 0;
	if(!finder.FindNextFile()) lastfile=1;
	CString strname = finder.GetFileName();
	sprintf(fname, "%s", strname.LockBuffer());
	return fname;
}

#endif // PSINS_VC_AFX_HEADER

//***************************  function AlignCoarse  *********************************/
CVect3 Alignsb(const CVect3 &wmm, const CVect3 &vmm, double lat)
{
//	if(latitude>PI/2)  latitude = asin(dot(wmm,vmm)/norm(wmm)/norm(vmm))/DEG;
	double T11, T12, T13, T21, T22, T23, T31, T32, T33;
	double cl = cos(lat), tl = tan(lat), nn;
	CVect3 wbib = wmm / norm(wmm),  fb = vmm / norm(vmm);
	T31 = fb.i,				 T32 = fb.j,			 	T33 = fb.k;
	T21 = wbib.i/cl-T31*tl,	 T22 = wbib.j/cl-T32*tl,	T23 = wbib.k/cl-T33*tl;		nn = sqrt(T21*T21+T22*T22+T23*T23);  T21 /= nn, T22 /= nn, T23 /= nn;
	T11 = T22*T33-T23*T32,	 T12 = T23*T31-T21*T33,		T13 = T21*T32-T22*T31;		nn = sqrt(T11*T11+T12*T12+T13*T13);  T11 /= nn, T12 /= nn, T13 /= nn;
	CMat3 Cnb(T11, T12, T13, T21, T22, T23, T31, T32, T33);
	return m2att(Cnb);
}

CVect3 Alignsb(const CVect3 &wmm, const CVect3 &vmm, const CVect3 &pos)
{
	return Alignsb(wmm, vmm, pos.i);
}


//***************************  CAlignsb  *********************************/
CAlignsb::CAlignsb(double lat0, double tk0)
{
	Init(lat0, tk0);
}

CAlignsb::CAlignsb(const CVect3 &pos0, double tk0)
{
	Init(pos0, tk0);
}

void CAlignsb::Init(double lat0, double tk0)
{
	Init(CVect3(lat0,0,0), tk0);  tk=tk0;
}

void CAlignsb::Init(const CVect3 &pos0, double tk0)
{
	this->pos0 = pos0; this->tk0 = tk = tk0;  yaw0 = INF; wmm = vmm = eb = db = O31;
	eth.Update(pos0);
}

void CAlignsb::SetYaw(double yaw0)
{
	this->yaw0 = yaw0;
}

CQuat CAlignsb::Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts)
{
	for(int i=0; i<nSamples; i++) {
		wmm += *pwm,  vmm += *pvm;  tk += ts;
	}
	att = Alignsb(wmm, vmm, pos0);
	if(yaw0<INFp5) att.k=yaw0;
	double _T = 1.0/(tk-tk0);
	CMat3 Cbn = ~a2mat(att);
	eb = wmm*_T - Cbn*eth.wnie;  db = vmm*_T + Cbn*eth.gn;
	return (qnb=a2qua(att));
}

//***************************  CAligni0  *********************************/
CAligni0::CAligni0(const CVect3 &pos0, const CVect3 &vel0, int velAid0)
{
	Init(pos0, vel0, velAid0);
}

void CAligni0::Init(const CVect3 &pos0, const CVect3 &vel0, int velAid0)
{
	eth.Update(pos0);
	this->pos0 = pos0, this->vel0 = vel0, velAid = velAid0;
	tk = 0;
	t0 = t1 = 10, t2 = 0; 
	wmm = vmm = vib0 = vi0 = Pib01 = Pib02 = Pi01 = Pi02 = O31;
	qib0b = CQuat(1.0);
}

CQuat CAligni0::Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts, const CVect3 &vel)
{
	double nts = nSamples*ts;
	imu.Update(pwm, pvm, nSamples, ts);
	wmm = wmm + imu.phim;  vmm = vmm + imu.dvbm;
	// vtmp = qib0b * (vm + 1/2 * wm X vm)
	CVect3 vtmp = qib0b*imu.dvbm;
	// vtmp1 = qni0' * [dvn+(wnin+wnie)Xvn-gn] * ts;
	tk += nts;
	CMat3 Ci0n = pos2Cen(CVect3(eth.pos.i,eth.wie*tk,0.0));
	CVect3 vtmp1 = Ci0n*(-eth.gn*nts);
	if(velAid>0)
	{
		CVect3 dv=vel-vel0;  vel0 = vel;
		if(velAid==1)		vtmp1 += Ci0n*dv;				// for GPS vn-aided
		else if(velAid==2)	vtmp -= qib0b*dv+imu.phim*vel;	// for OD vb-aided
	}
	// Pib02 = Pib02 + vib0*ts, Pi02 = Pi02 + vi0*ts
	vib0 = vib0 + vtmp,		 vi0 = vi0 + vtmp1;
	Pib02 = Pib02 + vib0*nts, Pi02 = Pi02 + vi0*nts;
	//
	if(++t2>3*t0)
	{
		t0 = t1, Pib01 = tmpPib0, Pi01 = tmpPi0;
	}
	else if(t2>2*t0 && t1==t0)
	{
		t1 = t2, tmpPib0 = Pib02, tmpPi0 = Pi02;
	}
	//
	qib0b = qib0b*rv2q(imu.phim);
	// qnb=qni0*qiib0*qib0b
	qnbsb = a2qua(Alignsb(wmm, vmm, eth.pos.i));
	if(t2<100)
	{
		qnb0 = qnb = CQuat(1.0);
	}
	else if(t2<1000)
	{
		qnb0 = qnb = qnbsb;
	}
	else
	{
		CQuat qi0ib0 = a2qua(dv2att(Pi01, Pi02, Pib01, Pib02));
		qnb0 = (~m2qua(pos2Cen(CVect3(eth.pos.i,0.0,0.0))))*qi0ib0;
		qnb = (~m2qua(Ci0n))*qi0ib0*qib0b;
	}
	return qnb;
}

//***************************  CAligni0fit*********************************/
CAligni0fit::CAligni0fit(void)
{
}

CAligni0fit::CAligni0fit(const CVect3 &pos0)
{
	Init(pos0);
}

void CAligni0fit::Init(const CVect3 &pos0)
{
	eth.Update(pos0);
	this->pos0 = pos0;
	tk = imu.tk = 0;
	vib0 = pib0 = vn0 = vnt = xyzt = O31;
	qib0b = qnb = qnbsb = CQuat(1.0);  qn0i0 = ~m2qua(pos2Cen(CVect3(eth.pos.i,0.0,0.0)));
	double G=glv.g0*eth.cl/glv.wie/glv.wie;
	pfit4.Init(0.0, 4); pfit4.SetP(100.0,100.0,100*G,100*G,100*G); pfit4.inHk = 0;
}

CQuat CAligni0fit::Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts)
{
	tk = imu.Update(pwm, pvm, nSamples, ts);
	// vib0/pib0 update
	CVect3 vib0tmp = vib0 + qib0b*imu.dvbm;
	pib0 += (vib0+vib0tmp)*imu.nts/2.0;  vib0 = vib0tmp;
	double wt=glv.wie*tk;
	pfit4.SetHk(1.0, wt, wt*wt/2.0, wt-sin(wt), 1.0-cos(wt)-wt*wt/2.0);  pfit4.Update(pib0);
	// qib0b update
	qib0b = qib0b*rv2q(imu.phim);
	// qnb=qni0*qiib0*qib0b
	qnbsb = a2qua(Alignsb(imu.swmm, imu.svmm, eth.pos.i));
	if(tk<1.0)			{	qnb0 = qnb = CQuat(1.0); }
	else if(tk<10.0)	{	qnb0 = qnb = qnbsb; }
	else {
		double tk1=tk/2.0;
		CVect3 pt=pib0t(tk), p1=pib0t(tk1);
		CQuat qi0ib0 = a2qua(dv2att(pi0t(tk), pi0t(tk1), pt-pfit4.Xkv[1]*glv.wie*tk-pfit4.Xkv[0], p1-pfit4.Xkv[1]*glv.wie*tk1-pfit4.Xkv[0]));
		qnb0 = (~m2qua(pos2Cen(CVect3(eth.pos.i,0.0,0.0))))*qi0ib0;
		CMat3 Ci0n = pos2Cen(CVect3(eth.pos.i,eth.wie*tk,0.0));  CQuat qnib0=(~m2qua(Ci0n))*qi0ib0;
		qnb = qnib0*qib0b;
		vn0 = -(qn0i0*qi0ib0*pfit4.Xkv[1]*glv.wie);
		vnt = qnib0*(vib0-glv.wie*((sin(wt)-wt)*pfit4.Xkv[4]+(1-cos(wt))*pfit4.Xkv[3]+wt*pfit4.Xkv[2]+pfit4.Xkv[1]));
		xyzt = qnib0*(pib0-pt);
	}
	return qnb;
}

CVect3 CAligni0fit::pi0t(double t)
{
    double wt = glv.wie*t;
    return CVect3(1.0-cos(wt), wt-sin(wt), eth.tl*wt*wt/2.0)*glv.g0*eth.cl/glv.wie/glv.wie;  // *glv.g0*eth.cl/glv.wie/glv.wie
}

CVect3 CAligni0fit::pib0t(double t)
{
    double wt = glv.wie*t;
	pfit4.SetHk(1.0, wt, wt*wt/2.0, wt-sin(wt), 1.0-cos(wt)-wt*wt/2.0); 
	return pfit4.eval(t);
}

//***************************  CAlignkf  *********************************/
CAlignkf::CAlignkf(void)
{
}

CAlignkf::CAlignkf(double ts):CSINSGNSS(15, 6, ts)
{
	CSINSGNSS::Init(CSINS(O31,O31,O31));
}

CAlignkf::CAlignkf(const CSINS &sins0, double ts):CSINSGNSS(15, 6, ts)
{
	Init(sins0);
}

void CAlignkf::Init(const CSINS &sins0)
{
	CSINSTDKF::Init(sins0);  sins.mvnT = 0.1;
	Pmax.Set2(fDEG3(30.0), fXXX(50.0), fdPOS(1.0e4), fDPH3(10.0), fMG3(10.0));
	Pmin.Set2(fXXZU(1.0,10.0, SEC), fXXX(0.001), fdPOS(0.01), fDPH3(0.001), fUG3(1.0));
	Pk.SetDiag2(fXXZU(1.10,10.0, DEG), fXXX(1.0), fdPOS(100.0), fDPH3(0.05), fUG3(100.0));
	Qt.Set2(fDPSH3(0.001), fUGPSHZ3(1.0), fXXX(0.0), fXXX(0.0), fXXX(0.0));
	Xmax.Set(fINF9, fDPH3(0.1), fMG3(1.0));
	Rt.Set2(fXXX(0.1), fdPOS(100.0));
	Rmax = Rt*100;  Rmin = Rt*0.01;  Rb = 0.5;
	FBTau.Set(fXXX(0.1), fXXX(0.1), fINF3, fXXX(1.1), fXXX(1.1));
	pos0 = sins.pos;  qnb = sins.qnb;
}

int CAlignkf::Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts, int nSteps)
{
	int res=TDUpdate(pwm, pvm, nSamples, ts, nSteps);
	if(sins.mvnk==0 && norm(sins.wnb)<0.1*glv.dps) {
		*(CVect3*)&Zk.dd[0] = sins.mvn;
		SetMeasFlag(0007);
		sins.pos = pos0;
	}
	qnb = sins.qnb;
	return res;
}

int CAlignkf::Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts, const CVect3 &vnr, int nSteps)
{
	int res=TDUpdate(pwm, pvm, nSamples, ts, nSteps);
	if(norm(vnr)>0)
	{
		*(CVect3*)&Zk.dd[0] = sins.vn-vnr;
		if(norm(sins.wnb)<1.1*glv.dps)
			SetMeasFlag(0007);
	}
	qnb = sins.qnb;
	return res;
}

//***************************  CAligntrkang  *********************************/
CAligntrkang::CAligntrkang(double ts, double vel00, double wz00, double dyaw00):CSINSGNSS(15, 4, ts)
{
	vel0 = vel00, wz0 = wz00, dyaw0 = dyaw00;
	Init(CSINS(O31,O31,O31));
}

void CAligntrkang::Init(const CSINS &sins0)
{
	CSINSGNSS::Init(sins0);
	Hk(3,6) = Hk(4,7) = Hk(5,8) = 0.0;  Hk(3,2) = 1.0;  // phi-meas
	Pmin.Set2(0.1*glv.min, 0.1*glv.min, 10*glv.min, 0.01, 0.01, 0.01, 1.0 / glv.Re, 1.0 / glv.Re, 0.1,
		1*DPH, 1*DPH, 1*DPH, 10.0*glv.ug, 10.0*glv.ug, 10.0*glv.ug);
	Pk.SetDiag2(10.0*DEG, 10.*DEG, 10.0*DEG, 10.0, 10.0, 10.0, 100.0 / glv.Re, 100.0 / glv.Re, 100.0,
		100.0*DPH, 100.0*DPH, 100.0*DPH, 1.0*glv.mg, 1.0*glv.mg, 1.0*glv.mg);
	Qt.Set2(1*glv.dpsh, 1*glv.dpsh, 1*glv.dpsh, 100.0*glv.ugpsHz, 100.0*glv.ugpsHz, 100.0*glv.ugpsHz, 0.0, 0.0, 0.0,
		0.0*glv.dphpsh, 0.0*glv.dphpsh, 0.0*glv.dphpsh, 0.0*glv.ugpsh, 0.0*glv.ugpsh, 0.0*glv.ugpsh);
	Rt.Set2(1.0, 1.0, 1.0, 1.0*DEG);
	Rmax = Rt * 100;  Rmin = Rt*0.01;  Rb = 0.9f;
	FBTau.Set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0);
	qnb = sins.qnb;
	cntYawOK = 0;
	velPre = yawPre = 0.0;
}

int CAligntrkang::Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts, const CVect3 &vnr, int nSteps)
{
	double vel=normXY(vnr), wz, yaw, dyaw;
	if(vel>1.0e-3)
	{
		wz = fabs(sins.wnb.k);
		if(wz<wz0) {  // vel meas
			*(CVect3*)&Zk.dd[0] = sins.vn-vnr;
			SetMeasFlag(0007);
		}
		yaw = atan2(-vnr.i,vnr.j);	dyaw = fabs(diffYaw(yaw,yawPre));
		if(velPre>vel0 && vel>vel0 && wz<wz0 && dyaw<dyaw0) {  // yaw meas
			Zk.dd[3] = -diffYaw(sins.att.k, yaw);
			if(fabs(Zk.dd[3])>20*DEG) {  // yaw init
				cntYawOK = 0;
				SetYaw(yaw); sins.vn = vnr;
				Pset.dd[0] = Pset.dd[1] = pow2(10*DEG); Pset.dd[2] = pow2(20*DEG);
				Pset.dd[3] = Pset.dd[4] = Pset.dd[5] = pow2(10.0);
			}
			else {
				cntYawOK ++;
				SetMeasFlag(0010);
			}
		}
		velPre = vel;
		yawPre = yaw;
	}
	TDUpdate(pwm, pvm, nSamples, ts, nSteps);
	qnb = sins.qnb;
	return cntYawOK;
}

//***************************  class CAlignsv  ********************************/
#ifdef PSINS_RMEMORY

CAlignsv::CAlignsv(void)
{
}

CAlignsv::CAlignsv(double ts):CAlignkf(ts)
{
	pMem = 0;
}

CAlignsv::CAlignsv(const CVect3 &pos, double ts, double T2, double T1):CAlignkf(ts)
{
	pMem = 0;
	Init(pos, ts, T2, T1);
}

void CAlignsv::Init(const CVect3 &pos, double ts, double T2, double T1)
{
	if(T1<10.0) T1 = 10.0;
	if(T1>T2/2 || T1<30.0) T1 = T2/2;
	t = tk = 0.0; this->ts = ts; this->T2 = T2; this->T1 = T1;
	alnkfinit = 0;
	alni0.Init(pos);
	Deletep(pMem);
	pMem = new CRMemory((int)(T1/ts+10), 6*sizeof(double));
}

CAlignsv::~CAlignsv()
{
	Deletep(pMem);
}

int CAlignsv::Update(const CVect3 *pwm, const CVect3 *pvm, int nSteps)
{
	double wvm[6];
	*(CVect3*)&wvm[0] = *pwm, *(CVect3*)&wvm[3] = *pvm;
	pMem->push((const unsigned char*)wvm);  // surport only single sub-sample
	if(t<T1) {  // align_i0
		alni0.Update((CVect3*)&wvm[0], (CVect3*)&wvm[3], 1, ts);
		qnb = alni0.qnb;
		tk += ts;  t += ts;
	}
	else {      // align_kf
		if(!alnkfinit) {
			alnkfinit = 1;  tk = 0.0;
			CAlignkf::Init(CSINS(alni0.qnb0, O31, alni0.pos0));  sins.imu=alni0.imu;
		}
		for(int i=0; i<2; i++) {
			if(pMem->pop((unsigned char*)wvm)) {
				CAlignkf::Update((CVect3*)&wvm[0], (CVect3*)&wvm[3], 1, ts, nSteps);
				tk += ts;  t += ts;
			}
		}
	}
	return tk>T2;
}

#endif // PSINS_RMEMORY for CAlignrv

//***************************  CAligntf  *********************************/
CAligntf::CAligntf(double ts):CSINSGNSS(19, 6, ts)
{
	CSINSGNSS::Init(CSINS(O31,O31,O31));
}

CAligntf::CAligntf(const CSINS &sins0, double ts):CSINSGNSS(19, 6, ts)
{
	Init(sins0);
}

void CAligntf::Init(const CSINS &sins0)
{
	// states: 0-2:phi,3-5:dvn,6-8:mu,9-11:eb,12-14:db,15-17:lv,18:dT   mu=C^bm_bs
	CSINSGNSS::Init(sins0);
	mu = lvMINS = O31;
	dtMINSdelay = 0.0;
	Hk.SetMat3(0,3,I33);
	Hk.SetMat3(3,0,I33), Hk.SetMat3(3,6,O33);
	Pmax.Set2(fDEG3(30.0), fXXX(50.0), fDEG3(10.0), fDPH3(10.0), fMG3(10.0), fXXX(10), 1.0);
	Pmin.Set2(fPHI(0.1,1.0), fXXX(0.001), fMIN3(0.1), fDPH3(0.00), fUG3(0.0), fXXX(0.01), 0.001);
	Pk.SetDiag2(fPHI(100.0,300.0), fXXX(1.0), fDEG3(3.0), fDPH3(100.0), fMG3(0.0), fXXX(1.0), 0.1);
	Qt.Set2(fDPSH3(0.1), fUGPSHZ3(10.0), fXXX(0.0), fXXX(0.0), fXXX(0.0), fXXX(0.0), 0.0);
	Rt.Set2(fXXX(0.1), fMIN3(3.0));
	Rmax = Rt*10000;  Rmin = Rt*0.01;  Rb = 0.5;
	FBTau.Set(fXXX(0.1), fXXX(0.1), fXXX(0.1), fXXX(1.1), fINF3, fXXX(1.1), 1.0);
}

void CAligntf::SetFt(int nnq)
{
	CSINSGNSS::SetFt(19);
	Ft.SetMat3(0,6,O33);
	Ft.SetMat3(3,6,O33);
	Ft.SetMat3(6,3,O33,O33);		// state6-8:dpos->mu
}

void CAligntf::SetHk(int nnq)
{
	CMat3 CW=sins.Cnb*askew(sins.web);
	Hk.SetMat3(0,15,-CW),      Hk.SetClmVect3(0,18,-sins.an);			// Zdvn
	Hk.SetMat3(3,6,-sins.Cnb), Hk.SetClmVect3(3,18,-sins.Cnb*sins.wib);	// Zdatt
}

void CAligntf::Feedback(int nnq, double fbts)
{
	CKalman::Feedback(nq, fbts);
	sins.qnb -= *(CVect3*)&FBXk.dd[0];  sins.vn -= *(CVect3*)&FBXk.dd[ 3];  mu += *(CVect3*)&FBXk.dd[6];
	sins.eb  += *(CVect3*)&FBXk.dd[9];	sins.db += *(CVect3*)&FBXk.dd[12];  // 0-14 phi,dvn,mu,eb,db
	lvMINS += *(CVect3*)&FBXk.dd[15];	// 15-17 lever
	dtMINSdelay += FBXk.dd[18];			// 18 dt
}

int CAligntf::Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts, int nSteps)
{
	int res=TDUpdate(pwm, pvm, 1, ts, nSteps);
	sins.lever(lvMINS);
	avpi.Push(sins, 1);
	return res;
}

void CAligntf::SetMeasVnAtt(const CVect3 &vnMINS, const CVect3 &attMINS)
{
	if(!IsZero(vnMINS))
	{
		if(avpi.Interp(dtMINSdelay)) {
			*(CVect3*)&Zk.dd[0] = avpi.vn - vnMINS;
			SetMeasFlag(00007);
		}
	}
	if(!IsZero(attMINS))
	{
		if(avpi.Interp(dtMINSdelay)) {
			*(CVect3*)&Zk.dd[3] = qq2phi(a2qua(avpi.att)*rv2q(-mu), a2qua(attMINS));
			SetMeasFlag(00070);
		}
	}
}

#ifdef PSINS_IO_FILE
void CAligntf::operator<<(CFileRdWt &f)
{
	f<<sins.att<<sins.vn<<mu<<sins.eb<<sins.db  // 1-15
		<<lvMINS<<dtMINSdelay <<kftk; // 16-20
}
#endif

CUartPP::CUartPP(int frameLen0, unsigned short head0)
{
	popIdx = 0;
	pushIdx = 1;
	unsigned char *p = (unsigned char*)&head0;
	head[0] = p[1], head[1] = p[0];
//	*(unsigned short*)head = head0;
	frameLen = frameLen0;
	csflag = 1, cs0 = 4, cs1 = frameLen-1, css = 2;
	overflow = getframe = 0;  safeBytes = 0;
}

BOOL CUartPP::checksum(const unsigned char *pc)
{
	chksum = 0;
	if(csflag==0) return 1;
	for(int i=cs0; i<=cs1; i++)
	{
		if(csflag==1)	chksum += pc[i];
		else if(csflag==2)  chksum ^= pc[i];
	}
	return chksum==pc[css];
}

inline int CUartPP::nextIdx(int idx)
{
	return idx >= UARTBUFLEN - 1 ? 0 : idx + 1;
}

int CUartPP::push(const unsigned char *buf0, int len)
{
	int overflowtmp = 0;
	for (int i=0; i<len; i++)
	{
		buf[pushIdx] = buf0[i];
		pushIdx = nextIdx(pushIdx);
		if (pushIdx==popIdx) {
			overflow++;
			overflowtmp++;
			popIdx = nextIdx(popIdx);
		}
	}
	return overflowtmp;
}

int CUartPP::pop(unsigned char *buf0)
{
	int getframetmp = 0;
	while(1)
	{
		if((pushIdx>popIdx&&popIdx+frameLen<pushIdx) || (pushIdx<popIdx&&popIdx+frameLen<pushIdx+UARTBUFLEN))
		{
			int popIdx1 = nextIdx(popIdx);
			if(buf[popIdx]==head[0] && buf[popIdx1]==head[1])
			{
				if(!buf0) buf0=&popbuf[0];
				for (int i=0; i<frameLen; i++)
				{
					buf0[i] = buf[popIdx];
					popIdx = nextIdx(popIdx);
				}
				if(checksum(buf0))  // checksum
				{
					getframe++;
					getframetmp = 1;
					break;
				}
				else
					popIdx = popIdx1; 
			}
			else
			{
				popIdx = popIdx1; 
			}
		}
		else
			break;
	}
	return getframetmp;
}

#ifdef PSINS_IO_FILE
BOOL CUartPP::safeWrite(CFileRdWt &f, int safeBytes0)
{
	BOOL newfile=0;
	if(safeBytes>safeBytes0 || (f.rwf==NULL)) {
		f.~CFileRdWt();  f.Init(time2fname());
		safeBytes = 0;
		newfile = 1;
	}
	fwrite(popbuf, frameLen, sizeof(char), f.rwf);
	safeBytes += frameLen;
	return newfile;
}
#endif

unsigned char chksum8(const unsigned char *pc, int len)
{
	unsigned char sum=0;
	for(const unsigned char *pend=&pc[len]; pc<pend; pc++) sum += *pc;
	return sum;
}

unsigned short chksum16(const unsigned short *ps, int len)
{
	unsigned short sum=0;
	for(const unsigned short *pend=&ps[len]; ps<pend; ps++) sum += *ps;
	return sum;
}

int makefrm(void *buf, ...)  // makefrm(buf, &int_data,4, &double_data,8, &wm,24, NULL)
{
	unsigned char *pc=(unsigned char*)buf;  int total=0;
	va_list vl;
	va_start(vl, buf);  
	for(int i=0; i<100; i++) {
		char *p=(char *)va_arg(vl,char*);  int len=va_arg(vl, int);
		if(p==NULL) break;
		for(int k=0; k<len; k++) { *pc++=*p++; total++; }
	}
	va_end(vl);
	return total;
}

double flt22db(float f1, float f2)
{
#define fltNEG 5.0e6F
	double db=f1;
	db += f2;	
	if(f1>=fltNEG) { db = -(db-fltNEG); }
	return db;
}

void db2flt2(double db, float *pf1, float *pf2)
{
	if(db>=0) {
		*pf1 = (float)((int)db);  *pf2 = (float)(db-*pf1);
	}
	else {
		db = -db;
		*pf1 = (float)((int)db);  *pf2 = (float)(db-*pf1);
		*pf1 += fltNEG;  // if negative, >=fltNEG
	}
}

int iga2flt(float *pf, const double *pwm, const double *pvm, const double *pgps, const double *pavp, const double *pt)
{
	float *pf0=pf;
	if(pwm) {
		*pf++=(float)*pwm++, *pf++=(float)*pwm++, *pf++=(float)*pwm;	// gyro
		*pf++=(float)*pvm++, *pf++=(float)*pvm++, *pf++=(float)*pvm;	// acc
	}
	if(pgps) {
		*pf++=(float)*pgps++, *pf++=(float)*pgps++, *pf++=(float)*pgps++;	// gps-ve/n/u
		db2flt2(*pgps++/DEG, &pf[0], &pf[1]);  pf+=2;						// gps-lat
		db2flt2(*pgps++/DEG, &pf[0], &pf[1]);  pf+=2;  *pf++=(float)*pgps;	// gps-lon/hgt
	}
	if(pavp) {
		*pf++=(float)*pavp++, *pf++=(float)*pavp++, *pf++=(float)*pavp++;	// att
		*pf++=(float)*pavp++, *pf++=(float)*pavp++, *pf++=(float)*pavp++;	// vn
		db2flt2(*pavp++/DEG, &pf[0], &pf[1]);  pf+=2;						// avp-lat
		db2flt2(*pavp++/DEG, &pf[0], &pf[1]);  pf+=2;  *pf++=(float)*pavp;	// avp-lon/hgt
	}
	if(pt) {
		db2flt2(*pt++, &pf[0], &pf[1]);  pf+=2;
	}
	return pf-pf0;  // count of float
}

//***************************  class CUartPP  *********************************/
#ifdef PSINS_CONSOLE_UART

#pragma message("  PSINS_CONSOLE_UART")

//***************************  class CConUart  *********************************/
CUartPP *pUart=NULL;
CFileRdWt *pfileUart=NULL;
void onExit(void) {
	if(pfileUart) pfileUart->~CFileRdWt();
}

CConUart::CConUart(void)
{
	hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
}

void CConUart::Init(int COMi, int BaudRate, int frameLen, unsigned short head)
{
	uart = CUartPP(frameLen, head);  uart.csflag = 0;  pUart = &uart;  pfileUart = &fraw;
	ps = (PSINSBoard*)&uart.popbuf[0];
	hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTitle("PSINS开发板上位机测试软件(by YGM@NWPU)");
	if(!m_SerialPort.InitPort(GetConsoleWindow(), comi=COMi, BaudRate))
	{
		printf("\nCOM%d open failed!\n\n", COMi);  exit(0);
	}
	m_SerialPort.StartMonitoring();
//	atexit(fraw.~CFileRdWt());
	atexit(onExit);
	cmd[0]=cmd[1]='#'; cmdstr[0]=cmdstr[1]='#', cmdstr[2]=0;
	cmdlen = 3;
}

BOOL CConUart::getUart(void)
{
	static int iflush=0;
	BOOL res = uart.pop();
	if(res) {
		if(!fraw.rwf) {
			fraw.Init(time2fname());
			gotorc(1);
			printf("\n\t原始数据保存\tCOM%d ---> %s", comi, fraw.fname);
		}
		fraw<<uart;
		if(iflush++==100) { iflush=0; fflush(fraw.rwf); }
	}
	else {
		Sleep(50);
	}
	sendUart();
	return res;
}
	
void CConUart::dispUart(int itv)
{
	static PSINSBoard ps1={0};
	if(ps->gpsstat>0.9) memcpy(&ps1, ps, sizeof(PSINSBoard));
	static int itv0=0;
	if(itv<1) itv=1;
	if(itv0++%itv) return;
	gotorc(4);
	printf("\n\t开发板时间(s)\t%10.3f \n\n\t陀螺(deg/s)\t%10.3f%10.3f%10.3f\t \n\t加表(m/s^2)\t%10.3f%10.3f%10.3f\t",
		ps->t, ps->gx,ps->gy,ps->gz, ps->ax,ps->ay,ps->az);
	printf("\n\t地磁(uT)\t%10.3f%10.3f%10.3f\t \n\t气压(mbar)\t%10.3f\t",
		ps->magx,ps->magy,ps->magz, ps->bar);
	printf("\n\n\t姿态角(deg)\t%10.3f%10.3f%10.3f\t \n\t速度(m/s)\t%10.3f%10.3f%10.3f\t \n\t位置(deg|m)\t%12.6lf%12.6lf%10.3f\t",
		ps->pch,ps->rll,ps->yaw, ps->ve,ps->vn,ps->vu, (double)ps->lon0+ps->lon1,(double)ps->lat0+ps->lat1,ps->hgt);
	printf("\n\n\tGPS速度(m/s)\t%10.3f%10.3f%10.3f\t \n\tGPS位置(deg|m)\t%12.6lf%12.6lf%10.3f\t",
		ps1.gpsve,ps1.gpsvn,ps1.gpsvu, (double)ps1.gpslon0+ps1.gpslon1,(double)ps1.gpslat0+ps1.gpslat1,ps1.gpshgt);
	printf("\n\tGPS卫星数.PDOP\t%10.3f\t \n\tGPS时延(s)\t%10.3f\t", ps1.gpsstat,ps1.gpsdly);
	printf("\n\n\t温度(℃)\t%10.3f\t", ps->tmp);
	printf("\n\n\t接收帧数\t%8d\t\n\t缓存溢出(byte)\t%8d\t", pUart->getframe, pUart->overflow);
	printf("\n\n\t*发送命令(%c%c)\t%s \t\t\t\t\t\t\t",	cmd[0],cmd[1],&cmdstr[0]);
}

void CConUart::gotorc(SHORT row, SHORT clm)
{
	COORD xy; xy.X=clm, xy.Y=row;	// (x,y)=(0,0) is left-up corner
	SetConsoleCursorPosition(hConsole, xy);
	CONSOLE_CURSOR_INFO  cci;	cci.dwSize=1, cci.bVisible=FALSE;
	SetConsoleCursorInfo(hConsole, &cci);
}

void CConUart::sendUart(void)
{
	if(!kbhit()) return;
	char c=getch(), c1;
	if(c=='l') {
		FILE *f=fopen(".\\Data\\pbcommand.txt","rt");
		if(f) {
			for(cmdlen=0; cmdlen<80; cmdlen++) {
				fscanf(f, "%c", &c1);  cmdstr[cmdlen]=c1;
				if(cmdlen>0 && c1=='#')
					break;
			}
			cmdstr[cmdlen]='#';  cmdstr[cmdlen+1] = '\n';  cmdstr[cmdlen+2] = 0;
			cmd[0]='l'; cmd[1]='#'; fclose(f); }
		else { sprintf(&cmdstr[1], "命令文件pbcommand.txt读取错误！"); }
	}
	else if(cmd[0]=='l' && c=='s') {
		cmd[0] = 'L';  cmd[1] = 's';
		m_SerialPort.WriteToPort(cmdstr, cmdlen+2);
	}
	else {
		cmd[0] = cmd[1] = '#';
	}
}

//***************************  class CComUart  *********************************/
CComUart::CComUart(void)
{
	hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
}

void CComUart::Init(char *fsetting)
{
	char info[256];
	FILE *f=fopen(fsetting, "rt");
	if(!f) { printf("Can't open setting file %s", fsetting); exit(0); }
	int baudrate, header, bSwap, frameLen=0;
	fscanf(f, "%d %d %x %d %s", &comi, &baudrate, &header, &bSwap, info);
	for(int i=0; i<80; i++) {
		char type;
		do { fscanf(f, "%c", &type); type=tolower(type);
		} while(!(type=='i' || type=='u' || type=='f' || type=='n' || type=='#'));
		uarts[i].type = type;
		if(uarts[i].type=='#') break;
		fscanf(f, "%d %c %lf %s", &uarts[i].len, &type, &uarts[i].scale, uarts[i].name);  uarts[i].len/=8;
		if(type=='/') uarts[i].scale = 1.0/uarts[i].scale;
		frameLen += uarts[i].len;
	}
	fclose(f);
	if(i>=80 || frameLen>=UARTFRMLEN) { printf("Frame length is too large (%d) !", frameLen); exit(0); }
	uart = CUartPP(frameLen, header);  uart.csflag = 0;  pUart = &uart;  pfileUart = &fraw;
	Init(comi, baudrate, bSwap);
}

void CComUart::Init(int comi, int baudrate, int bSwap)
{
	hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTitle("PSINS串口上位机测试软件(by YGM@NWPU)");
	if(!m_SerialPort.InitPort(GetConsoleWindow(), comi, baudrate))	{
		printf("\nCOM%d open failed!\n\n", comi);  exit(0);
	}
	m_SerialPort.StartMonitoring();  sendLen = 0;
	bSwapbytes = bSwap;
	atexit(onExit);
	gotorc(1);	printf("\n\tNo UART data received ...", comi, fraw.fname);
}

BOOL CComUart::getUart(void)
{
	static int iflush=0;
	BOOL res = uart.pop();
	if(res) {
//		if(uart.safeWrite(fraw, 10*1024*1024)) {  10 MBytes
		if(uart.safeWrite(fraw, 10*1024*1024)) {
			gotorc(1);	printf("\n\t原始数据保存\tCOM%d ---> %s", comi, fraw.fname);
		}
		if(iflush++==100) { iflush=0; fflush(fraw.rwf); }
	}
	else {
		Sleep(10);
	}
	return res;
}

void CComUart::dispUart(int itv)
{
	static int itv0=0;
	if(itv<1) itv=1;
	if(itv0++%itv) return;
	unsigned char buf[UARTFRMLEN], *pbuf=buf;
	memcpy(buf, uart.popbuf, uart.frameLen);
	gotorc(4);  printf("\t");
	for(int i=0; i<80; i++){
		if(uarts[i].type=='#') break;
		if(bSwapbytes&&uarts[i].len<=8) swapbytes(pbuf, uarts[i].len);
		double val;
		if(uarts[i].type=='i') {
			if(uarts[i].len==1)  { val=*(char*) pbuf*uarts[i].scale; }
			if(uarts[i].len==2)  { val=*(short*)pbuf*uarts[i].scale; }
			if(uarts[i].len==3)  { val=i24i32((const unsigned char*)pbuf)*uarts[i].scale; }
			if(uarts[i].len==4)  { val=*(int*)  pbuf*uarts[i].scale; }
		}
		else if(uarts[i].type=='u') {
			if(uarts[i].len==1)  { val=*(unsigned char*) pbuf*uarts[i].scale; }
			if(uarts[i].len==2)  { val=*(unsigned short*)pbuf*uarts[i].scale; }
			if(uarts[i].len==4)  { val=*(unsigned int*)  pbuf*uarts[i].scale; }
		}
		else if(uarts[i].type=='f') {
			if(uarts[i].len==4)  { val=*(float*) pbuf*uarts[i].scale; }
			if(uarts[i].len==8)  { val=*(double*)pbuf*uarts[i].scale; }
		else if(uarts[i].type=='n')
			val=uarts[i].len;
		}
		pbuf += uarts[i].len;
		if(i) { if(val>1.0e10) val=1.0e10; else if(val<-1.0e10) val=-1.0e10; }
		printf(uarts[i].name, val);  printf("\n\t");
	}
	printf("接收帧数\t%8d\t缓存溢出(bytes)\t%8d\t", pUart->getframe, pUart->overflow);
}

void CComUart::sendUart(uchar *buf, int len, int waitms)
{
	m_SerialPort.WriteToPort(buf, len);
	sendLen += len;
	printf("SendLen = %d\n", sendLen);
	if(waitms>0) Sleep(waitms);
}

void CComUart::gotorc(SHORT row, SHORT clm)
{
	COORD xy; xy.X=clm, xy.Y=row;	// (x,y)=(0,0) is left-up corner
	SetConsoleCursorPosition(hConsole, xy);
	CONSOLE_CURSOR_INFO  cci;	cci.dwSize=1, cci.bVisible=FALSE;
	SetConsoleCursorInfo(hConsole, &cci);
}

#endif  // PSINS_CONSOLE_UART

unsigned short swap16(unsigned short ui16)
{
	unsigned char *p = (unsigned char*)&ui16, c;
	c = p[0]; p[0] = p[1]; p[1] = c;
	return ui16;
}

unsigned char* swap24(unsigned char* puc3, unsigned char* pres)
{
	static unsigned char resc[3];
	if (pres == NULL) pres = resc;
	pres[0] = puc3[2]; pres[1] = puc3[1];  pres[2] = puc3[0];
	return pres;
}

unsigned int swap32(unsigned int ui32)
{
	unsigned char *p = (unsigned char*)&ui32, c;
	c = p[0]; p[0] = p[3]; p[3] = c;
	c = p[1]; p[1] = p[2]; p[2] = c;
	return ui32;
}

unsigned long swap64(unsigned long ui64)
{
	unsigned char *p = (unsigned char*)&ui64, c;
	c = p[0]; p[0] = p[7]; p[7] = c;
	c = p[1]; p[1] = p[6]; p[6] = c;
	c = p[2]; p[2] = p[5]; p[5] = c;
	c = p[3]; p[3] = p[4]; p[4] = c;
	return ui64;
}

unsigned char* swapbytes(unsigned char *pc, int nBytes)
{
	unsigned char c;
	switch(nBytes)
	{
	case 2: c=pc[0], pc[0]=pc[1], pc[1]=c; break;
	case 3: c=pc[0], pc[0]=pc[2], pc[2]=c; break;
	case 4: c=pc[0], pc[0]=pc[3], pc[3]=c; c=pc[1], pc[1]=pc[2], pc[2]=c; break;
	case 8: c=pc[0], pc[0]=pc[7], pc[7]=c; c=pc[1], pc[1]=pc[6], pc[6]=c;
			c=pc[2], pc[2]=pc[5], pc[5]=c; c=pc[3], pc[3]=pc[4], pc[4]=c; break;
	}
	return pc;
}

unsigned char* int24(unsigned char *pchar3, int int32)
{
	unsigned char *p = (unsigned char*)&int32;
	*pchar3++ = *p++, *pchar3++ = *p++, *pchar3 = *p;
	return pchar3-2;
}

int diffint24(const unsigned char *pc1, const unsigned char *pc0)
{
	int i1, i0;
	unsigned char *p1 = (unsigned char*)&i1, *p0 = (unsigned char*)&i0;
	*p1++ = 0, *p1++ = *pc1++, *p1++ = *pc1++, *p1 = *pc1;
	*p0++ = 0, *p0++ = *pc0++, *p0++ = *pc0++, *p0 = *pc0;
	return (i1 - i0) / 256;
}

int i24i32(const unsigned char *pchar3)  // int24 to int32
{
	int int32;
	unsigned char *p = (unsigned char*)&int32;
	*(++p) = *pchar3++, *(++p) = *pchar3++, *(++p) = *pchar3;
	return int32/256;
}

#ifdef PSINS_psinsassert

#pragma message("  psinsassert();")

BOOL psinsassert(BOOL b)
{
	int res;

	if(b)	{
		res = 1;
	}
	else	{
		res = 0;
	}
	return res;
}

#endif

double r2dm(double r)	// rad to deg/min, eg. 1234.56 = 12deg+34.56min
{
	int sgn=1;
	if(r<0.0) { r=-r; sgn=0; }
	double deg = r/DEG;
	int ideg = (int)deg;
	double dm = ideg*100 + (deg-ideg)*60.0;
	return sgn ? dm : -dm;
}

double dm2r(double dm)
{
	int sgn=1;
	if(dm<0.0) { dm=-dm; sgn=0; }
	int ideg = (int)(dm/100);
	double r = ideg*DEG + (dm-ideg*100)*(DEG/60);
	return sgn ? r : -r;
}

BOOL logtrigger(int n, double f0)
{
	static int trigger_count=0;
	if(++trigger_count==n || f0>EPS || f0<-EPS)
	{
		trigger_count = 0;
		return TRUE;
	}
	return FALSE;
}

inline BOOL IsZero(double f, double eps)
{
	return (f<eps && f>-eps);
}

// determine the sign of 'val' with the sensitivity of 'eps'
int sign(double val, double eps)
{
	int s;

	if(val<-eps)
	{
		s = -1;
	}
	else if(val>eps)
	{
		s = 1;
	}
	else
	{
		s = 0; 
	}
	return s;
}

// set double value 'val' between range 'minVal' and 'maxVal'
double range(double val, double minVal, double maxVal)
{
	double res;

	if(val<minVal)
	{ 
		res = minVal; 
	}
	else if(val>maxVal)	
	{ 
		res = maxVal; 
	}
	else
	{ 
		res = val;
	}
	return res;
}

double  maxn(const double *pd, int n)
{
	double m=-INF; const double *pn=&pd[n];
	for(; pd<pn; pd++) {
		if(*pd>m) m=*pd;
	}
	return m;
}

double  minn(const double *pd, int n)
{
	double m=INF; const double *pn=&pd[n];
	for(; pd<pn; pd++) {
		if(*pd<m) m=*pd;
	}
	return m;
}

double norm1(const double *pd, int n)
{
	double n1=0.0; const double *pn=&pd[n];
	for(; pd<pn; pd++) {
		if(*pd>0.0) n1+=*pd; else n1-=*pd;
	}
	return n1;
}

double norm(const double *pd, int n)
{
	double n2=0.0; const double *pn=&pd[n];
	for(; pd<pn; pd++) {
		n2 += *pd*(*pd);
	}
	return sqrt(n2);
}

double normInf(const double *pd, int n)
{
	double ninf=0.0; const double *pn=&pd[n];
	for(; pd<pn; pd++) {
		if(*pd>ninf) ninf=*pd; else if(-*pd>ninf) ninf=-*pd;
	}
	return ninf;
}

double attract(double f, double th, double center)
{
	f -= center;
	if(f>-th && f<th) {
		th = f/th;  th *= th;
		f *= th;
	}
	return (f+center);
}

double polyval(const double *p, int order, double x)
{
	double y=p[0];
	for(int k=1; k<=order; k++)  y = y*x + p[k];
	return y;
}

double atan2Ex(double y, double x)
{
	double res;

	if((sign(y)==0) && (sign(x)==0))
	{
		res = 0.0;
	}
	else
	{
		res = atan2(y, x);
	}
	return res;
}

double diffYaw(double yaw, double yaw0)
{
	double dyaw = yaw-yaw0;
	if(dyaw>=PI) dyaw-=_2PI;
	else if(dyaw<=-PI) dyaw+=_2PI;
	return dyaw;
}

double MKQt(double sR, double tau)
{
	return sR*sR*2.0/tau;
}

CVect3 MKQt(const CVect3 &sR, const CVect3 &tau)
{
	return CVect3(sR.i*sR.i*2.0/tau.i, sR.j*sR.j*2.0/tau.j, sR.k*sR.k*2.0/tau.k);
}

double unixt2gpst(double ut, int leap)
{
	return fmod(ut+leap-315964800.0, 86400.0);
}

BOOL chkhdr(const char *str, const char *hdr)
{
	while(1) {
		if(*hdr=='\0') return 1;
		if(*hdr++!=*str++) return 0;
	}
}

double randn(double mu, double sigma)
{
#define NSUM 25
static double ssgm = sqrt(NSUM/12.0);
	double x = 0;
	for (int i=0; i<NSUM; i++)
	{
		x += (double)rand();
	}
	x /= RAND_MAX;
	x -= NSUM/2.0;
	x /= ssgm;		x *= sigma;
	return x+mu;
}

CVect3 randn(const CVect3 &mu, const CVect3 &sigma)
{
	return CVect3(randn(mu.i,sigma.i), randn(mu.j,sigma.j), randn(mu.k,sigma.k));
}

CVect randn(const CVect &mu, const CVect &sigma)
{
	CVect vtmp(mu.row,mu.clm);
	const double *pmu=&mu.dd[0], *pEnd=&mu.dd[mu.rc], *psigma=&sigma.dd[0];
	for(double *p=&vtmp.dd[0]; pmu<pEnd; p++,pmu++,psigma++) *p=randn(*pmu,*psigma);
	return vtmp;
}

CMat3 randn(const CMat3 &mu, const double &sigma)
{
	CMat3 mtmp;
	const double *pmu=&mu.e00, *pEnd=&mu.e22;
	for(double *p=&mtmp.e00; pmu<=pEnd; p++,pmu++) *p=randn(*pmu,sigma);
	return mtmp;
}

CMat randn(const CMat &mu, const double &sigma)
{
	CMat mtmp(mu.row,mu.clm);
	const double *pmu=&mu.dd[0], *pEnd=&mu.dd[mu.rc];
	for(double *p=&mtmp.dd[0]; pmu<pEnd; p++,pmu++) *p=randn(*pmu,sigma);
	return mtmp;
}

int* deci(int i, int *pi)
{
#define di_len 6
	static int di[di_len];   // 'i=12345' => 'di[0]=1,di[1]=2,di[2]=3,di[3]=4,di[4]=5'
	int k;
	for(k=0; k<di_len; k++) {  // decode
		di[k] = i;  i /= 10;  di[k] -= i*10;
		if(i==0) break;
	}
	for(int k1=0; k1<=k/2; k1++) {  // reverse
		int tmp=di[k1]; di[k1]=di[k-k1], di[k-k1]=tmp;
	}
	if(pi) {
		for(int j=0; j<=k; j++) pi[j]=di[j];
	}
	return di;
}

#ifdef PSINS_EXTERN_C_EXAMPLE

CSINS sinsExample;

extern "C" void psinsInit0(const double *att0, const double *vn0, const double *pos0, double t0)
{
	sinsExample.Init(*(const CVect3*)att0, *(const CVect3*)vn0, *(const CVect3*)pos0, t0);
}

extern "C" void psinsUpdate0(const double *gyro, const double *acc, int n, double ts)
{
	sinsExample.Update((CVect3*)gyro, (CVect3*)acc, n, ts);
}

extern "C" void psinsUpdatef(const float *gyro, const float *acc, int n, double ts)
{
	CVect3 wm[5], vm[5];
	double *pwm=&wm[0].i, *pvm=&vm[0].i, *pEnd=&wm[n].i;
	for(; pwm<pEnd; pwm++,pvm++,gyro++,acc++) { *pwm=*gyro, *pvm=*acc; }
	sinsExample.Update(wm, vm, n, ts);
}

extern "C" void psinsOut0(double *att, double *vn, double *pos, double *tk)
{
	*(CVect3*)att = sinsExample.att, *(CVect3*)vn = sinsExample.vn, *(CVect3*)pos = sinsExample.pos;
	*tk = sinsExample.tk;
}

#endif // PSINS_EXTERN_C_EXAMPLE

void add(CVect3 &res, const CVect3 &v1, const CVect3 &v2)
{
	res.i=v1.i+v2.i, res.j=v1.j+v2.j, res.k=v1.k+v2.k;
}

void add(CMat3 &res, const CMat3 &m1, const CMat3 &m2)
{
	res.e00=m1.e00+m2.e00, res.e01=m1.e01+m2.e01, res.e02=m1.e02+m2.e02; 
	res.e10=m1.e10+m2.e10, res.e11=m1.e11+m2.e11, res.e12=m1.e12+m2.e12; 
	res.e20=m1.e20+m2.e20, res.e21=m1.e21+m2.e21, res.e22=m1.e22+m2.e22; 
}

void add(CVect &res, const CVect &v1, const CVect &v2)
{
	const double *p1=v1.dd, *p2=v2.dd, *pEnd=&v1.dd[v1.rc];
	for(double *p=res.dd; p1<pEnd; p++,p1++,p2++) *p=*p1+*p2;
}

void add(CMat &res, const CMat &m1, const CMat &m2)
{
	const double *p1=m1.dd, *p2=m2.dd, *pEnd=&m1.dd[m1.rc];
	for(double *p=res.dd; p1<pEnd; p++,p1++,p2++) *p=*p1+*p2;
}

void sub(CVect3 &res, const CVect3 &v1, const CVect3 &v2)
{
	res.i=v1.i-v2.i, res.j=v1.j-v2.j, res.k=v1.k-v2.k;
}

void sub(CMat3 &res, const CMat3 &m1, const CMat3 &m2)
{
	res.e00=m1.e00-m2.e00, res.e01=m1.e01-m2.e01, res.e02=m1.e02-m2.e02; 
	res.e10=m1.e10-m2.e10, res.e11=m1.e11-m2.e11, res.e12=m1.e12-m2.e12; 
	res.e20=m1.e20-m2.e20, res.e21=m1.e21-m2.e21, res.e22=m1.e22-m2.e22; 
}

void sub(CVect &res, const CVect &v1, const CVect &v2)
{
	const double *p1=v1.dd, *p2=v2.dd, *pEnd=&v1.dd[v1.rc];
	for(double *p=res.dd; p1<pEnd; p++,p1++,p2++) *p=*p1-*p2;
}

void sub(CMat &res, const CMat &m1, const CMat &m2)
{
	const double *p1=m1.dd, *p2=m2.dd, *pEnd=&m1.dd[m1.rc];
	for(double *p=res.dd; p1<pEnd; p++,p1++,p2++) *p=*p1-*p2;
}

void cros(CVect3 &res, const CVect3 &v1, const CVect3 &v2)
{
	double
		i=v1.j*v2.k-v1.k*v2.j,
		j=v1.k*v2.i-v1.i*v2.k;
	res.k=v1.i*v2.j-v1.j*v2.i;
	res.i=i, res.j=j;
}

void dotmul(CVect3 &res, const CVect3 &v1, const CVect3 &v2)
{
	res.i=v1.i*v2.i, res.j=v1.j*v2.j, res.k=v1.k*v2.k;
}

void dotdiv(CVect3 &res, const CVect3 &v1, const CVect3 &v2)
{
	res.i=v1.i/v2.i, res.j=v1.j/v2.j, res.k=v1.k/v2.k;
}

void mul(CVect3 &res, const CVect3 &v, const double &f)
{
	res.i=v.i*f, res.j=v.j*f, res.k=v.k*f;
}

void mul(CMat3 &res, const CMat3 &m, const double &f)
{
	res.e00=m.e00*f, res.e01=m.e01*f, res.e02=m.e02*f;
	res.e10=m.e10*f, res.e11=m.e11*f, res.e12=m.e12*f;
	res.e20=m.e20*f, res.e21=m.e21*f, res.e22=m.e22*f;
}

void mul(CVect &res, const CVect &v, const double &f)
{
	res.row=v.row, res.clm=v.clm, res.rc=v.rc;
	const double *p1=v.dd, *pEnd=&v.dd[v.rc];
	for(double *p=res.dd; p1<pEnd; p++,p1++) *p=*p1*f;
}

void mul(CVect &res, const CMat &m, const CVect &v)
{
	res.row=res.rc=m.row, res.clm=1;
	double *p=res.dd, *pEnd=&res.dd[res.row]; const double *p1ij=m.dd, *p2End=&v.dd[v.rc];
	for(; p<pEnd; p++)
	{
		double f=0.0; const double *p2j=v.dd;
		for(; p2j<p2End; p1ij++,p2j++)	f += (*p1ij) * (*p2j);
		*p = f;
	}
}

void mul(CMat &res, const CMat &m, const double &f)
{
	res.row=m.row, res.clm=m.clm, res.rc=m.rc;
	const double *p1=m.dd, *pEnd=&m.dd[m.rc];
	for(double *p=res.dd; p1<pEnd; p++,p1++) *p=*p1*f;
}

void mul(CVect3 &res, const CMat3 &m, const CVect3 &v)
{
	res.i=m.e00*v.i+m.e01*v.j+m.e02*v.k, res.j=m.e10*v.i+m.e11*v.j+m.e12*v.k, res.k=m.e20*v.i+m.e21*v.j+m.e22*v.k;
}

void mul(CVect3 &res, const CVect3 &v, const CMat3 &m)
{
	res.i=m.e00*v.i+m.e10*v.j+m.e20*v.k, res.j=m.e01*v.i+m.e11*v.j+m.e21*v.k, res.k=m.e02*v.i+m.e12*v.j+m.e22*v.k;
}

void mul(CMat3 &res, const CMat3 &m1, const CMat3 &m2)
{
	if(&res==&m1||&res==&m2) {
		res=m1*m2;
	}
	else {
		res.e00 = m1.e00*m2.e00 + m1.e01*m2.e10 + m1.e02*m2.e20;
		res.e01 = m1.e00*m2.e01 + m1.e01*m2.e11 + m1.e02*m2.e21;
		res.e02 = m1.e00*m2.e02 + m1.e01*m2.e12 + m1.e02*m2.e22;
		res.e10 = m1.e10*m2.e00 + m1.e11*m2.e10 + m1.e12*m2.e20;
		res.e11 = m1.e10*m2.e01 + m1.e11*m2.e11 + m1.e12*m2.e21;
		res.e12 = m1.e10*m2.e02 + m1.e11*m2.e12 + m1.e12*m2.e22;
		res.e20 = m1.e20*m2.e00 + m1.e21*m2.e10 + m1.e22*m2.e20;
		res.e21 = m1.e20*m2.e01 + m1.e21*m2.e11 + m1.e22*m2.e21;
		res.e22 = m1.e20*m2.e02 + m1.e21*m2.e12 + m1.e22*m2.e22;
	}
}

void mul(CQuat &res, const CQuat &q1, const CQuat &q2)
{
	double 
		qq0 = q1.q0*q2.q0 - q1.q1*q2.q1 - q1.q2*q2.q2 - q1.q3*q2.q3,
		qq1 = q1.q0*q2.q1 + q1.q1*q2.q0 + q1.q2*q2.q3 - q1.q3*q2.q2,
		qq2 = q1.q0*q2.q2 + q1.q2*q2.q0 + q1.q3*q2.q1 - q1.q1*q2.q3;
	 res.q3 = q1.q0*q2.q3 + q1.q3*q2.q0 + q1.q1*q2.q2 - q1.q2*q2.q1;
	res.q0=qq0, res.q1=qq1, res.q2=qq2;
}

void mul(CVect3 &res, const CQuat &q, const CVect3 &v)
{
	double
		qq0 =         - q.q1*v.i - q.q2*v.j - q.q3*v.k,
		qq1 = q.q0*v.i           + q.q2*v.k - q.q3*v.j,
		qq2 = q.q0*v.j           + q.q3*v.i - q.q1*v.k,
		qq3 = q.q0*v.k           + q.q1*v.j - q.q2*v.i;
	res.i = -qq0*q.q1 + qq1*q.q0 - qq2*q.q3 + qq3*q.q2;
	res.j = -qq0*q.q2 + qq2*q.q0 - qq3*q.q1 + qq1*q.q3;
	res.k = -qq0*q.q3 + qq3*q.q0 - qq1*q.q2 + qq2*q.q1;
}

void _TT(CMat3 &mT, const CMat3 &m)
{
	mT.e00=m.e00, mT.e10=m.e01, mT.e20=m.e02;
	mT.e01=m.e10, mT.e11=m.e11, mT.e21=m.e12;
	mT.e02=m.e20, mT.e12=m.e21, mT.e22=m.e22;
}

void rv2q(CQuat &q, const CVect3 &rv)
{
	double n2 = rv.i*rv.i+rv.j*rv.j+rv.k*rv.k, f;
	if(n2<(PI/180.0*PI/180.0))	// 0.017^2 
	{
		double n4=n2*n2;
		q.q0 = 1.0 - n2*(1.0/rvF2) + n4*(1.0/rvF4);
		f = 0.5 - n2*(1.0/rvF3) + n4*(1.0/rvF5);
	}
	else
	{
		double n_2 = sqrt(n2)/2.0;
		q.q0 = cos(n_2);
		f = sin(n_2)/n_2*0.5;
	}
	q.q1=f*rv.i, q.q2=f*rv.j, q.q3=f*rv.k;
}

void qdelphi(CQuat &q, const CVect3 &phi)
{
	CQuat qtmp;
	rv2q(qtmp, phi);
	double
		  q0 = qtmp.q0*q.q0 - qtmp.q1*q.q1 - qtmp.q2*q.q2 - qtmp.q3*q.q3,
		  q1 = qtmp.q0*q.q1 + qtmp.q1*q.q0 + qtmp.q2*q.q3 - qtmp.q3*q.q2,
		  q2 = qtmp.q0*q.q2 + qtmp.q2*q.q0 + qtmp.q3*q.q1 - qtmp.q1*q.q3;
		q.q3 = qtmp.q0*q.q3 + qtmp.q3*q.q0 + qtmp.q1*q.q2 - qtmp.q2*q.q1;
	q.q0=q0; q.q1=q1; q.q2=q2;
}

void q2mat(CMat3 &Cnb, const CQuat &qnb)
{
	double	q11 = qnb.q0*qnb.q0, q12 = qnb.q0*qnb.q1, q13 = qnb.q0*qnb.q2, q14 = qnb.q0*qnb.q3, 
			q22 = qnb.q1*qnb.q1, q23 = qnb.q1*qnb.q2, q24 = qnb.q1*qnb.q3,     
			q33 = qnb.q2*qnb.q2, q34 = qnb.q2*qnb.q3,  
			q44 = qnb.q3*qnb.q3;
    Cnb.e00 = q11+q22-q33-q44,  Cnb.e01 = 2*(q23-q14),     Cnb.e02 = 2*(q24+q13),
	Cnb.e10 = 2*(q23+q14),      Cnb.e11 = q11-q22+q33-q44, Cnb.e12 = 2*(q34-q12),
	Cnb.e20 = 2*(q24-q13),      Cnb.e21 = 2*(q34+q12),     Cnb.e22 = q11-q22-q33+q44;
}

void m2att(CVect3 &att, const CMat3 &Cnb)
{
//	att.i = asinEx(Cnb.e21);
	float e21=(float)Cnb.e21;
	if(e21<-1.0) e21=1.0; else if(e21>1.0) e21=1.0;  att.i = asinf(e21);
//	att.j = atan2Ex(-Cnb.e20, Cnb.e22);
	float e20=(float)Cnb.e20, e22=(float)Cnb.e22;
	att.j = (e20>1.0e-10||e20<-1.0e-10 || e22>1.0e-10||e22<-1.0e-10) ? atan2f(-e20, e22) : 0.0;
//	att.k = atan2Ex(-Cnb.e01, Cnb.e11);
	float e01=(float)Cnb.e01, e11=(float)Cnb.e11;
	att.k = (e01>1.0e-10||e01<-1.0e-10 || e11>1.0e-10||e11<-1.0e-10) ? atan2f(-e01, e11) : 0.0;
}

void AXbt(CVect3 &res, const CMat3 &A, const CVect3 &X, const CVect3 &b, const double &t)
{
	double  i = A.e00*X.i+A.e01*X.j+A.e02*X.k + b.i*t,
		    j = A.e10*X.i+A.e11*X.j+A.e12*X.k + b.j*t;
		res.k = A.e20*X.i+A.e21*X.j+A.e22*X.k + b.k*t;
	res.i=i, res.j=j;
}

void ClassSizeDisp(int i)
{
#define ClsSize(xxx)	printf("\t"#xxx":%10d\n", sizeof(xxx))
	printf("Size of each class (in bytes):\n");
	ClsSize(CVect);
	ClsSize(CMat);
	ClsSize(CMaxMinn);
	ClsSize(CAligni0);
	ClsSize(CAligni0fit);
	ClsSize(CSINS);
	ClsSize(CKalman);
	ClsSize(CSINSTDKF);
	ClsSize(CSINSGNSS);
	ClsSize(CAutoDrive);
	ClsSize(CRAvar);
	if(i>0) printf("\tclassXXX: %10d\n", i);
	exit(0);
}
