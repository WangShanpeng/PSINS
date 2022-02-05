
/* PSINS(Precise Strapdown Inertial Navigation System) C++ algorithm source file PSINS.cpp

Copyright(c) 2015-2021, by YanGongmin, All rights reserved.
Northwestern Polytechnical University, Xi'an, P.R.China.
Date: 17/02/2015, 19/07/2017, 11/12/2018, 27/12/2019, 12/12/2020, 22/11/2021, 27/01/2022
*/

#include "PSINS.h"

const CVect3 O31(0.0), One31(1.0), Ipos(1.0/RE,1.0/RE,1.0), posNWPU=LLH(34.246048,108.909664,380); //NWPU-Lab-pos
const CQuat  qI(1.0,0.0,0.0,0.0);
const CMat3  I33(1,0,0, 0,1,0, 0,0,1), O33(0,0,0, 0,0,0, 0,0,0), One33(1.0);
const CVect  On1(MMD,0.0), O1n=~On1, Onen1(MMD,1.0);
const CGLV   glv;
int			 psinslasterror = 0;
int			 psinsstack0 = 0, psinsstacksize = 0;

//***************************  class CGLV  *********************************/
CGLV::CGLV(double Re, double f, double wie0, double g0)
{
	this->Re = Re; this->f = f; this->wie = wie0; this->g0 = g0;
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
}

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

double norm(const CVect3 &v)
{
	return sqrt(v.i*v.i + v.j*v.j + v.k*v.k);
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

double dot(const CVect3 &v1, const CVect3 &v2)
{
	return (v1.i*v2.i + v1.j*v2.j + v1.k*v2.k);
}

CVect3 dotmul(const CVect3 &v1, const CVect3 &v2)
{
	return CVect3(v1.i*v2.i, v1.j*v2.j, v1.k*v2.k);
}

CQuat rv2q(const CVect3 &rv)
{
#define F1	(   2 * 1)		// define: Fk=2^k*k! 
#define F2	(F1*2 * 2)
#define F3	(F2*2 * 3)
#define F4	(F3*2 * 4)
#define F5	(F4*2 * 5)
	double n2 = rv.i*rv.i+rv.j*rv.j+rv.k*rv.k, c, f;
	if(n2<(PI/180.0*PI/180.0))	// 0.017^2 
	{
		double n4=n2*n2;
		c = 1.0 - n2*(1.0/F2) + n4*(1.0/F4);
		f = 0.5 - n2*(1.0/F3) + n4*(1.0/F5);
	}
	else
	{
		double n_2 = sqrt(n2)/2.0;
		c = cos(n_2);
		f = sin(n_2)/n_2*0.5;
	}
	return CQuat(c, f*rv.i, f*rv.j, f*rv.k);
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

double sinAng(const CVect3 &v1, const CVect3 &v2)
{
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

CVect3 sort(const CVect3 &v)
{
	CVect3 vtmp=v;
	if(vtmp.i<vtmp.j) swapt(vtmp.i,vtmp.j,double);
	if(vtmp.i<vtmp.k) swapt(vtmp.i,vtmp.k,double);
	if(vtmp.j<vtmp.k) swapt(vtmp.j,vtmp.k,double);
	return vtmp;
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

CVect3 CQuat::operator-(CQuat &quat) const
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

void normlize(CQuat *q)
{
	double nq=sqrt(q->q0*q->q0+q->q1*q->q1+q->q2*q->q2+q->q3*q->q3);
	q->q0 /= nq, q->q1 /= nq, q->q2 /= nq, q->q3 /= nq;
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
	return att;
}

CVect3 dv2att(const CVect3 &va1, const CVect3 &va2, const CVect3 &vb1, const CVect3 &vb2)
{
	CVect3 a=va1*va2, b=vb1*vb2, aa=a*va1, bb=b*vb1;
	if(IsZero(va1)||IsZero(a)||IsZero(aa)||IsZero(vb1)||IsZero(b)||IsZero(bb)) return O31;
	CMat3 Ma(va1/norm(va1),a/norm(a),aa/norm(aa)), Mb(vb1/norm(vb1),b/norm(b),bb/norm(bb));
	return m2att((~Ma)*(Mb));  // return C^a_b
}

CVect3 vn2att(const CVect3 &vn)
{
	double vel = normXY(vn);
	if(vel<1.0e-6) return O31;
    return CVect3(atan2(vn.k, vel), 0, atan2(-vn.i, vn.j));
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

void CMat3::SetRow(int i, CVect3 &v)
{
	double *p=&e00+i*3;
	*p=v.i, *(p+1)=v.j, *(p+2) = v.k;
}

void CMat3::SetClm(int i, CVect3 &v)
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
#ifdef MAT_COUNT_STATISTIC
	#pragma message("  MAT_COUNT_STATISTIC")
	if(iMax<++iCount) iMax = iCount;
#endif
}
	
CMat::CMat(int row0, int clm0)
{
#ifdef MAT_COUNT_STATISTIC
	if(iMax<++iCount) iMax = iCount;
#endif
	row=row0; clm=clm0; rc=row*clm;
}

CMat::CMat(int row0, int clm0, double f)
{
#ifdef MAT_COUNT_STATISTIC
	if(iMax<++iCount) iMax = iCount;
#endif
	row=row0; clm=clm0; rc=row*clm;
	for(double *pd=dd, *pEnd=&dd[rc]; pd<pEnd; pd++)  *pd = f;
}

CMat::CMat(int row0, int clm0, double f, double f1, ...)
{
#ifdef MAT_COUNT_STATISTIC
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
#ifdef MAT_COUNT_STATISTIC
	if(iMax<++iCount) iMax = iCount;
#endif
	row=row0; clm=clm0; rc=row*clm;
	memcpy(dd, pf, rc*sizeof(double));
}

#ifdef MAT_COUNT_STATISTIC
int CMat::iCount=0, CMat::iMax=0;
CMat::~CMat(void)
{
	iCount--;
}
#endif

void CMat::Clear(void)
{
	for(double *p=dd, *pEnd=&dd[rc]; p<pEnd; p++)	*p = 0.0;
}

CMat CMat::operator*(const CMat &m0) const
{
#ifdef MAT_COUNT_STATISTIC
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
#ifdef MAT_COUNT_STATISTIC
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
#ifdef MAT_COUNT_STATISTIC
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
#ifdef MAT_COUNT_STATISTIC
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
#ifdef MAT_COUNT_STATISTIC
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

void CMat::SetRowVect3(int i, int j, const CVect3 &v)
{
	*(CVect3*)&dd[i*clm+j] = v;
}

void CMat::SetDiagVect3(int i, int j, const CVect3 &v)
{
	double *p=&dd[i*clm+j];
	*p = v.i;  p += clm+1;
	*p = v.j;  p += clm+1;
	*p = v.k;
}

CVect3 CMat::GetDiagVect3(int i, int j)
{
	if(j==-1) j=i;
	CVect3 v;
	double *p=&dd[i*clm+j];
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

CVect CMat::GetClm(int j) const
{
	CVect v(row, 1);
	const double *p1=&dd[j], *pEnd=&dd[rc];
	for(double *p=v.dd; p1<pEnd; p++,p1+=clm) *p = *p1;
	return v;
}

void CMat::ZeroRow(int i)
{
	for(double *p=&dd[i*clm],*pEnd=p+clm; p<pEnd; p++) *p = 0.0;
	return;
}

void CMat::ZeroClm(int j)
{
	for(double *p=&dd[j],*pEnd=&dd[rc]; p<pEnd; p+=clm) *p = 0.0;
	return;
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

double normInf(CMat &m)
{
	double n1=0.0;
	for(double *p=m.dd,*pEnd=&m.dd[m.rc]; p<pEnd; p++)
	{
		if(*p>n1)	 n1 = *p;
		else if(-*p>n1) n1 = -*p;
	}
	return n1;
}

CVect diag(const CMat &m)
{
	int row1 = m.row+1;
	CVect vtmp(m.row,1);
	double *p=vtmp.dd, *pEnd=&vtmp.dd[vtmp.row];
	for(const double *p1=m.dd; p<pEnd; p++, p1+=row1)	*p = *p1;
	return vtmp;
}

void RowMul(CMat &m, const CMat &m0, const CMat &m1, int r)
{
	psinsassert(m0.clm==m1.row);
	int rc0=r*m0.clm;
	double *p=&m.dd[rc0], *pEnd=p+m0.clm; const double *p0=&m0.dd[rc0], *p0End=p0+m0.clm, *p1j=m1.dd;
	for(; p<pEnd; p++)
	{
		double f=0.0; const double *p0j=p0, *p1jk=p1j++;
		for(; p0j<p0End; p0j++,p1jk+=m1.clm)	 f += (*p0j) * (*p1jk);
		*p = f;
	}
}

void RowMulT(CMat &m, const CMat &m0, const CMat &m1, int r)
{
	psinsassert(m0.clm==m1.clm);
	int rc0=r*m0.clm;
	double *p=&m.dd[rc0], *pEnd=p+m0.clm; const double *p0=&m0.dd[rc0], *p0End=p0+m0.clm, *p1jk=m1.dd;
	for(; p<pEnd; p++)
	{
		double f=0.0; const double *p0j=p0;
		for(; p0j<p0End; p0j++,p1jk++)	 f += (*p0j) * (*p1jk);
		*p = f;
	}
}

CMat diag(const CVect &v)
{
#ifdef MAT_COUNT_STATISTIC
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
	va_list vl;
	va_start(vl, f);
	for(int i=0, rc=row>clm?row:clm; i<rc; i++)
	{ if(f>2*INF) break;  dd[i] = f;  f = va_arg(vl, double);	}
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

double norm(const CVect &v)
{
	const double *p=v.dd, *pEnd=&v.dd[v.rc];
	double f=0.0;
	for(; p<pEnd; p++)  { f += (*p)*(*p); }
	return sqrt(f);
}

double normInf(const CVect &v)
{
	const double *p=v.dd, *pEnd=&v.dd[v.rc];
	double f=0.0;
	for(; p<pEnd; p++)  { if(*p>f) f=*p; else if(-*p>f) f=-*p; }
	return f;
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

CVect3 CVect::GetVect3(int i)
{
	return *(CVect3*)&dd[i]; 
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

double CRAvar::operator()(int k)
{
	return Rmaxflag[k] ? INF : sqrt(R0[k]);
}

//***************************  class CVAR  ***********************************/
CVAR::CVAR(int imax0, double data0)
{
	imax = min(imax0, VARMAX);
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
	for (int i = 1; i < clm; i++) {
		pData[i] = pData[i - 1] + row;
	}
	pd = pData[clm-1]+row;  Sx = pd + clm;  Sx2 = Sx + clm;  mx = Sx2 + clm;  stdx = mx + clm;
	stdsf = sqrt((row - 1) / (row - 2.0));
	Reset();
}

CVARn::~CVARn(void)
{
	Delete(pData);
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
	for (int i = 0; i < clm; i++)
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
	for (int i = 0; i < clm; i++)
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
CMaxMin::CMaxMin(int cnt00, int pre00, double f0)
{
	max0=f0, min0=-f0, maxpre0=f0, minpre0=-f0;
	maxCur=f0, minCur=-f0, maxpreCur=f0, minpreCur=-f0;
	maxRes=f0, minRes=-f0, maxpreRes=f0, minpreRes=-f0;
	cntCur=cnt0=cnt00;
	cntpreCur = (pre00<=0||pre00>=cnt00) ? cnt00/2 : cnt0-pre00;
	flag = 0;
}

int CMaxMin::Update(double f)
{
	flag=0;
	if(maxCur<f) maxCur=f; else if(minCur>f) minCur=f;
	if(maxpreCur<f) maxpreCur=f; else if(minpreCur>f) minpreCur=f;
	if(--cntCur<=0) {
		maxRes=maxCur; minRes=minCur; maxCur=minCur=f; cntCur=cnt0; flag=1;
	}
	if(--cntpreCur<=0) {
		maxpreRes=maxpreCur; minpreRes=minpreCur; maxpreCur=minpreCur=f; cntpreCur=cnt0; flag=-1;
	}
	return flag;
}

//***************************  class CMaxMinn  *********************************/
CMaxMinn::CMaxMinn(int n0, int cnt00, int pre00, double f0)
{
	n = n0;
	CMaxMin mm0(cnt00, pre00, f0);
	for(int i=0; i<n; i++)	mm[i] = mm0;
	flag = 0;
}

int CMaxMinn::Update(double f, ...)
{
	CMaxMin *pm=&mm[0];
	va_list vl;
	va_start(vl, f);
	for(int i=0; i<n; i++,pm++)
	{
		pm->Update(f);
		f = va_arg(vl, double);
	}
	va_end(vl);
	return flag = mm[0].flag;
}

int CMaxMinn::Update(const CVect3 &v)
{
	mm[0].Update(v.i); mm[1].Update(v.j); mm[2].Update(v.k);
	return flag = mm[0].flag;
}

int CMaxMinn::Update(const CVect3 &v1, const CVect3 &v2)
{
	mm[0].Update(v1.i); mm[1].Update(v1.j); mm[2].Update(v1.k);
	mm[3].Update(v2.i); mm[4].Update(v2.j); mm[5].Update(v2.k);
	return flag = mm[0].flag;
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
	Hk = CMat(nr,nq,0.0);  Fading = CMat(nr,nq,1.0); zfdafa = 0.1;
	Qt = Pmin = Xk = CVect(nq,0.0);  Xmax = Pmax = CVect(nq,INF);  Pset = CVect(nq,-INF);
	Zk = CVect(nr,0.0);  Rt = CVect(nr,INF); rts = CVect(nr,1.0);  Zfd = CVect(nr,0.0); Zfd0 = Zmax = CVect(nr,INF);
	RtTau = Rmax = CVect(nr,INF); measstop = measlost = Rmin = Rb = Rstop = CVect(nr,0.0); Rbeta = CVect(nr,1.0);
	SetRmaxcount(5);
	FBTau = FBMax = FBOne = FBOne1 = CVect(nq,INF); FBXk = FBTotal = CVect(nq,0.0);
	kfcount = measflag = measflaglog = 0;  SetMeasMask(3,nr0);
}

void CKalman::SetRmaxcount(int cnt)
{
	for(int i=0; i<nr; i++) { Rmaxcount[i]=0, Rmaxcount0[i]=cnt; }
}

void CKalman::TimeUpdate(double kfts, int fback)
{
	CMat Fk;
	kftk += kfts;  kfcount++;
	SetFt(nq);
	Fk = ++(Ft*kfts);  // Fk = I+Ft*ts
	Xk = Fk * Xk;
	Pk = Fk*Pk*(~Fk);  Pk += Qt*kfts;
	if(fback)  Feedback(nq, kfts);
	for(int i=0; i<nr; i++) {
		measlost.dd[i] += kfts;
		if(measstop.dd[i]>0.0) measstop.dd[i] -= kfts;
	}
}

void CKalman::SetMeasFlag(unsigned int flag)
{
	measflag = (flag==0) ? 0 : (measflag|flag);
}

void CKalman::SetMeasMask(int type, unsigned int mask)
{
	int m;
	if(type==1) measmask = mask;		// set mask 1
	else if(type==0) measmask &= ~mask;	// set mask 0
	else if(type==2) measmask |= mask;	// add mask 1
	else if(type==3) {					// set mask-LSB 1
		for(m=0; mask>0; mask--)  m |= 1<<(mask-1);
		SetMeasMask(1, m);
	}
}

void CKalman::SetMeasStop(double stop, unsigned int meas)
{
	measstop.SetBit(meas, stop);
}

void CKalman::SetRadptStop(double stop, unsigned int meas)
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

int CKalman::RAdaptive(int i, double r, double Pr)
{
	double rr=r*r-Pr;
	if(rr<Rmin.dd[i])	rr = Rmin.dd[i];
	if(rr>Rmax.dd[i])	{ Rt.dd[i]=Rmax.dd[i]; Rmaxcount[i]++; }  
	else				{ Rt.dd[i]=(1.0-Rbeta.dd[i])*Rt.dd[i]+Rbeta.dd[i]*rr; Rmaxcount[i]=0; }
	Rbeta.dd[i] = Rbeta.dd[i]/(Rbeta.dd[i]+Rb.dd[i]);   // beta = beta / (beta+b)
	int adptOK = (Rmaxcount[i]==0||Rmaxcount[i]>Rmaxcount0[i]) ? 1: 0;
	return adptOK;
}

void CKalman::RPkFading(int i)
{
	Zfd.dd[i] = Zfd.dd[i]*(1-zfdafa) + Zk.dd[i]*zfdafa;
	if(Zfd.dd[i]>Zfd0.dd[i] || Zfd.dd[i]<-Zfd0.dd[i])
		DVMDVafa(Fading.GetRow(i), Pk);
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
}

void CSINSTDKF::Init(const CSINS &sins0)
{
	sins = sins0;  kftk = sins.tk;
	Fk = Pk1 = CMat(nq,nq, 0.0);
	Pxz = Qk = Kk = tmeas = CVect(nr, 0.0);
	meantdts = 1.0; tdts = 0.0;
	maxStep = 2*(nq+nr)+3;
	TDReset();
	curOutStep = 0; maxOutStep = 1;
	timcnt0 = timcnt1 = 0, timcntmax = 100;  burden = 0.0;
}

void CSINSTDKF::TDReset(void)
{
	iter = -2;
	ifn = 0;	meanfn = O31;
	SetMeasFlag(0);
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
	meanfn = meanfn+sins.fn; ifn++;
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
			Fk = ++(Ft*tdts); // Fk = I+Ft*ts
			Qk = Qt*tdts;
			Xk = Fk*Xk;
//			RtFading(tdts);
			meantdts = tdts; tdts = 0.0;
		}
		else if(iter<nq)		// 0 -> (nq-1): Fk*Pk
		{
			int row=iter;
			RowMul(Pk1, Fk, Pk, row);
		}
		else if(iter<2*nq)		// nq -> (2*nq-1): Fk*Pk*Fk+Qk
		{
			int row=iter-nq;
			RowMulT(Pk, Pk1, Fk, row);
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
					Hi = Hk.GetRow(row);
					Pxz = Pk*(~Hi);
					Pz0 = (Hi*Pxz)(0,0);
					innovation = Zk(row)-(Hi*Xk)(0,0);
					adptOKi = 1;
					if(Rb.dd[row]>EPS && Rstop.dd[row]<EPS)
						adptOKi=RAdaptive(row, innovation, Pz0);
					double Pzz = Pz0 + Rt(row)/rts(row);
					Kk = Pxz*(1.0/Pzz);
				}
				else
				{
					measflag ^= flag;
					if(adptOKi && measstop.dd[row]<EPS && (innovation>-Zmax.dd[row]&&innovation<Zmax.dd[row]))
					{
						measRes |= flag;
						Xk += Kk*innovation;
						Pk -= Kk*(~Pxz);
						measlost.dd[row] = 0.0;
					}
					if(Zfd0.dd[row]<INFp5)
					{
						RPkFading(row);
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
	posGNSSdelay = vnGNSSdelay = yawGNSSdelay = dtGNSSdelay = dyawGNSS = -0.0;
	kfts = ts;  gnsslost = &measlost.dd[3];
	lvGNSS = O31;
	Hk(0,3) = Hk(1,4) = Hk(2,5) = 1.0;		// dvn
	Hk(3,6) = Hk(4,7) = Hk(5,8) = 1.0;		// dpos
	yawHkRow = yawHkRow0;
	if(yawHkRow>=6) Hk(yawHkRow,2) = 1.0;	// dyaw
	SetMeasMask(1, 077);
	SetMeasStop(1.0);
}

void CSINSGNSS::Init(const CSINS &sins0, int grade)
{
	CSINSTDKF::Init(sins0);
	sins.lever(-lvGNSS);  sins.pos = sins.posL;    // sins0.pos is GNSS pos
	avpi.Init(sins, kfts, 1);
	if(grade==0) {  // inertial grade
		Pmax.Set2(fDEG3(1.0),  fXXX(100.0),  fdPOS(1.0e6), fDPH3(0.5),  fMG3(1.0),
			fXXX(100.0), 0.5, fdKG9(1000.0,900.0), fdKA6(100.0,100.0));
		Pmin.Set2(fPHI(0.1,1.0),  fXXX(0.001),  fdPOS(.01),  fDPH3(0.001),  fUG3(10.0),
			fXXX(0.001), 0.0001, fdKG9(1.0,1.0), fdKA6(1.0,1.0));
		Pk.SetDiag2(fPHI(60,600),  fXXX(1.0),  fdPOS(100.0),  fDPH3(0.1),  fMG3(1.0),
			fXXX(1.0),  0.01,  fdKG9(100.0,90.0), fdKA6(10.0,10.0));
		Qt.Set2(fDPSH3(0.001),  fUGPSHZ3(1.0),  fOOO,  fOO6,
			fOOO, 0.0,  fOO9,  fOO6);
		//MarkovGyro(I31*1000.0, I31*0.01*DPH);  MarkovAcc(I31*1000.0, I31*1.0*UG);
		Xmax.Set(fINF9,  fDPH3(0.5),  fMG3(1.0),
			fXXX(10.0), 0.5,  fdKG9(1000.0,900.0),  fdKA6(1000.0,900.0));
		Rt.Set2(fXXZ(0.5,1.0),   fdLLH(10.0,30.0));
		Rmax = Rt*100;  Rmin = Rt*0.01;  Rb = 0.6;
		FBOne1.Set(fPHI(1,1), fXXX(0.1), fdLLH(1,3), fDPH3(0.01), fUG3(10), fXXX(0.1), 0.01, fINF9,  fINF6);
		FBTau.Set(fXX9(0.1),  fXX6(1.0),  fINF3, INF,  fINF9,  fINF6);
	}
	else if(grade==1) {  // MEMS grade
		Pmax.Set2(fDEG3(50.0),  fXXX(100.0),  fdPOS(1.0e6), fDPH3(5000.0),  fMG3(30.0),
			fXXX(100.0), 0.5, fdKG9(1000.0,900.0), fdKA6(100.0,100.0));
		Pmin.Set2(fPHI(1,1),  fXXX(0.0001),  fdPOS(.001),  fDPH3(0.001),  fUG3(1.0),
			fXXX(0.001), 0.0001, fdKG9(1.0,1.0), fdKA6(1.0,1.0));
		Pk.SetDiag2(fPHI(600,600),  fXXX(1.0),  fdPOS(100.0),  fDPH3(1000.0),  fMG3(10.0),
			fXXX(1.0),  0.01,  fdKG9(1000.0,90.0), fdKA6(10.0,10.0));
		Qt.Set2(fDPSH3(1.1),  fUGPSHZ3(10.0),  fOOO,  fOO6,
			fOOO, 0.0,  fOO9,  fOO6);
		Xmax.Set(fINF9,  fDPH3(3600.0),  fMG3(50.0),
			fXXX(10.0), 0.5,  fdKG9(1000.0,900.0),  fdKA6(1000.0,900.0));
		Rt.Set2(fXXZ(0.5,1.0),   fdLLH(10.0,30.0));
		Rmax = Rt*100;  Rmin = Rt*0.01;  Rb = 0.6;
		FBTau.Set(fXX9(0.1),  fXX6(1.0),  fINF3, INF,  fINF9,  fINF6);
	}
}

void CSINSGNSS::SetFt(int nnq)
{
	sins.etm();
	Ft.SetMat3(0,0,sins.Maa), Ft.SetMat3(0,3,sins.Mav), Ft.SetMat3(0,6,sins.Map), Ft.SetMat3(0,9,-sins.Cnb); 
	Ft.SetMat3(3,0,sins.Mva), Ft.SetMat3(3,3,sins.Mvv), Ft.SetMat3(3,6,sins.Mvp), Ft.SetMat3(3,12,sins.Cnb); 
						NULL, Ft.SetMat3(6,3,sins.Mpv), Ft.SetMat3(6,6,sins.Mpp);
	Ft.SetDiagVect3( 9, 9, sins._betaGyro);
	Ft.SetDiagVect3(12,12, sins._betaAcc);  // 0-14 phi,dvn,dpos,eb,db
	if(nnq>=18) NULL;						// 15-17 lever
	if(nnq>=19) NULL;						// 18 dt
	if(nnq==20) {
		Ft(2,19) = -sins.wib.k*sins.Cnb.e22;  // 19 dKGzz
	}
	else if(nnq==22) {
		CMat3 Cwz=-sins.wib.k*sins.Cnb;
		Ft.SetMat3(0,19, Cwz);				// 19-21 dKGz
	}
	else if(nnq>=28) {
		CMat3 Cwx=-sins.wib.i*sins.Cnb, Cwy=-sins.wib.j*sins.Cnb, Cwz=-sins.wib.k*sins.Cnb; 
		Ft.SetMat3(0,19, Cwx);  Ft.SetMat3(0,22, Cwy);  Ft.SetMat3(0,25, Cwz);  // 19-27 dKG
	}
	if(nnq>=34) {
		CMat3 Cfx= sins.fb.i *sins.Cnb, Cfy= sins.fb.j *sins.Cnb, Cfz= sins.fb.k *sins.Cnb;
		Cfz.e00=Cfy.e01,	Cfz.e01=Cfy.e02; 
		Cfz.e10=Cfy.e11,	Cfz.e11=Cfy.e12; 
		Cfz.e20=Cfy.e21,	Cfz.e21=Cfy.e22;
		Ft.SetMat3(3,28, Cfx);  Ft.SetMat3(3,31, Cfz);  // 28-33 dKA(xx,yx,zx, yy,zy, zz)
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
	sins.qnb -= *(CVect3*)&FBXk.dd[0];  sins.vn -= *(CVect3*)&FBXk.dd[ 3];  sins.pos -= *(CVect3*)&FBXk.dd[6];
	sins.eb  += *(CVect3*)&FBXk.dd[9];	sins.db += *(CVect3*)&FBXk.dd[12];  // 0-14 phi,dvn,dpos,eb,db
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
		sins.Kg = IdKGz*sins.Kg;		// 19-21 dKGz
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

void CSINSGNSS::SetMeasGNSS(const CVect3 &posgnss, const CVect3 &vngnss, double yawgnss, double qfactor)
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
	Delete(pmemImuGnss); Delete(pmemFusion); Delete(pmemFusion1);
	Delete(psmth);
	Delete(fins); Delete(fkf);
}

BOOL CPOS618::Load(char *fname, double t0, double t1)
{
	printf("--- Load data ---\n OK\n");
	CFileRdWt f(fname,-(int)(sizeof(ImuGnssData)/sizeof(double)));
	if(t0>EPS) f.load((int)(t0/ts));
	records = f.filesize()/sizeof(ImuGnssData);
	if(t1>EPS) {
		if(t1-1.0<t0) return FALSE;
		records = min(records,(int)((t1-t0)/ts));
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
	Rt.Set2(fXXZ(0.1,0.3),   fdLLH(0.1,0.3));
	Rmax = Rt*100;  Rmin = Rt*0.01;  Rb = 0.6;
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
	pIG=(ImuGnssData*)pmemImuGnss->get(records), pXP1 = (FXPT*)pmemFusion1->get(records);
	for(--iter,--pIG,--pXP,--pXP1; iter>=0; iter--,pIG--,pXP--,pXP1--)
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
	TDReset();  avpi.Init(sins,kfts,1);  SetMeasStop(2.0);
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
	SetMeasMask(1, 0777);
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
	Rt.Set2(fXXZ(0.5,1.0),   fdLLH(10.0,30.0), fXYZU(10,10,30,SEC));
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
	*(CVect3*)&Zk.dd[6] = sins.qnb - m2qua(cns.GetCns(qis, sins.pos, sins.tk, Cbs));
	SetMeasFlag(00700);
}

void CSINSGNSSCNS::SetMeasCNS(CVect3 &vqis)
{
	double q0=1.0-dot(vqis,vqis);
	q0 = q0>0 ? sqrt(q0) : 0.0;
	SetMeasCNS(CQuat(q0, vqis));
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
	Rt.Set2(fXXZ(0.2,0.6), fdLLH(10.0,30.0), fdLLH(1.1,1.0), fdLLH(10,10), 1.0*DEG);
	Rmax = Rt*100;  Rmin = Rt*0.01;  Rb = 0.5;
	FBTau.Set(fIII, fIII, fIII, fIII, fIII, fIII, fIII);
}

void CSINSGNSSDR::SetFt(int nnq)
{
	CSINSGNSS::SetFt(15);
	CMat3 MvkD = norm(sins.vn)*CMat3(-sins.Cnb.e02,sins.Cnb.e01,sins.Cnb.e00,
			-sins.Cnb.e12,sins.Cnb.e11,sins.Cnb.e10,-sins.Cnb.e22,sins.Cnb.e21,sins.Cnb.e20);
	Ft.SetMat3(18, 0, sins.Mpv*askew(sins.vn));
	Ft.SetMat3(18, 15, sins.Mpv*MvkD);
	Ft.SetMat3(18, 18, sins.Mpp);
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
	if(*gnsslost>5.0)
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
	odmeast = 0.0;  odmeasT = 1.0;  dS0 = 0.0;  distance = 0.0;
	lvOD = vnOD = O31;  ODKappa(CVect3(0,1,0));
	odVelOK = 0;
}

void CSINSGNSSOD::Init(const CSINS &sins0, int grade)
{
	CSINSGNSS::Init(sins0, grade);
	sins.lever(lvOD, &posOD, &vnOD);
	odmeasT = 1.0;
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
	if(dS<-1.0||dS>10.0) { dS = dS0; } dS0 = dS;  // if bad OD input 
	if(odmeast<EPS) {   // re-initialize
		IVno = IVni = Ifn = O31;
		IMv = odMlever = O33;
	}
	dS *= Kod;
	distance += dS;
	CVect3 dSn = sins.Cnb*(CVect3(Cbo.e01*dS,Cbo.e11*dS,Cbo.e21*dS)-sins.imu.phim*lvOD);  // dSn = Cnb*Cbo*dSb
	odmeast += sins.nts;
	if(odmeast>=odmeasT) {  // take measurement every (odmeasT)(s)
		odMphi = -askew(odmeast/2.0*Ifn+IVno);
		odMvn = odmeast*I33;
		odMkappa = CMat3( IMv.e02,-IMv.e01,-IMv.e00,
						  IMv.e12,-IMv.e11,-IMv.e10,
						  IMv.e22,-IMv.e21,-IMv.e20 );
		//odMlever = sins.Cnb*askew(-odmeast*sins.web);
		odZk = IVni - IVno; 
		odmeast = 0.0;
		return odVelOK=1;
	}
	else {
		IVno += dSn;  IVni += sins.vn*sins.nts; Ifn += sins.fn*sins.nts;
		IMv += dS*sins.Cnb;
		odMlever += sins.Cnb*askew(-sins.nts*sins.web);
		return odVelOK=0;
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
		<<dtGNSSdelay<<dyawGNSS <<kftk; // 47-39
}
#endif

//***************************  class CAutoDrive  *********************************/
CAutoDrive::CAutoDrive(void)
{
}

CAutoDrive::CAutoDrive(double ts):CSINSGNSSOD(18, 17, ts, 15)
{
	// 0-14: phi,dvn,dpos,eb,db; 15-17: Kappa;
	gnsslostdist = distlost = 0.0;
	odlost = &measlost.dd[6];  zuptlost = &measlost.dd[9];
	// Hk(0:5,:) ...		// 0-5: SINS/GNSS-dvn,dpos
	Hk.SetMat3(6, 3, I33);  // 6-8: SINS/OD-dvn
	Hk.SetMat3(9, 3, I33);  // 9-11: ZUPT
	Hk.SetMat3(12, 3, I33); // 12-14: NHC
	Hk(15,2) = 1.0;			// 15: SINS/GNSS-dyaw
	Hk(16,11) = 1.0;		// 16: WzHold (ZIHR)
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
		fXXZ(0.1,0.1), fXXZ(0.1,0.1), fXYZ(0.1,10.0,0.1), 1.0*DEG, 10.0*DPH);
	Rmax = Rt*100;  Rmin = Rt*0.01;  Rb.SetBit(0100077, 0.5);
	*(CVect3*)&Zmax.dd[0] = CVect3(2.0,2.0,0.5);
	FBTau = 1.0;
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
	if(sins.mmwb.flag==1) {
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
	if(wzhd.retres==3 && sins.fn.k>9.5) {
		Zk.dd[16] = wzhd.meanwpre-sins.eth.wnie.k;
		SetMeasFlag(0200000);
		Xk.dd[2] -= Zk.dd[16]*wzhd.T;
	}
}

void CAutoDrive::NHCtest(void)
{
	if(IsZero(sins.wnb.k,20*DPS)) {
		CMat3 Con = (~Cbo)*sins.Cbn;
		Hk.SetMat3(12,3,Con);
		*(CVect3*)&Zk.dd[12] = Con*sins.vn;
		SetMeasFlag(050000);
	}
}

int CAutoDrive::Update(const CVect3 *pwm, const CVect3 *pvm, double dS, int nn, double ts, int nSteps)
{
	int res = CSINSGNSSOD::Update(pwm, pvm, dS, nn, ts, nSteps);
	if(odVelOK) {
		Hk.SetMat3(6, 0, odMphi); Hk.SetMat3(6, 3, odMvn); Hk.SetMat3(6, 15, odMkappa);
		*(CVect3*)&Zk.dd[6] = odZk;  // SINS/OD-dvn
		SetMeasFlag(0700);
	}
	if(*gnsslost>10 && *odlost>10.0) ZUPTtest();
	ZIHRtest();
	if(*gnsslost>10 && *odlost>10 && *zuptlost>10) NHCtest();
	return res;
}

//***************************  class CSGOClbt  *********************************/
CSGOClbt::CSGOClbt(double ts):CSINSGNSSOD(26, 10, ts, 9)
{
	// 0-14: phi,dvn,dpos,eb,db; 15-17: lvGNSS; 18-20: Kappa; 21-23: lvOD; 24: dtGNSS; 25: dyawGNSS
	// Hk(0:5,:) ...								// 0-5: SINS/GNSS-dvn,dpos
	Hk.SetMat3(6, 3, I33);							// 6-8: SINS/OD-dvn
	Hk(yawHkRow,2) = 1.0; Hk(yawHkRow,25) = -1.0; 	// 9: SINS/GNSS-dyaw
	SetMeasMask(1, 01777);
}

void CSGOClbt::Init(const CSINS &sins0, int grade)
{
	CSINSGNSSOD::Init(sins0, grade);
	if(grade<0) return;
	Pmax.Set2(fDEG3(10.0), fXXX(50.0), fdPOS(1.0e4),
		fDPH3(3600), fMG3(10.0), fXXX(1.1), fKPP(1,0.01,1), fXXX(10.0), 0.1, 1*DEG);
	Pmin.Set2(fPHI(0.1,0.6), fXXX(0.001), fdPOS(0.01),
		fDPH3(0.1), fUG3(100), fXXZ(0.01,0.01), fKPP(0.01,0.001,0.01), fXXX(0.001), 0.0001, 0.01*DEG);
	Pk.SetDiag2(fDEG3(1.0), fXXX(1.0), fdPOS(100.0),
		fDPH3(10), fMG3(1.0), fXXZ(1.1,1.1), fKPP(0.1,0.01,1.1), fXXX(1.0), 0.01, 1.1*DEG);
	Qt.Set2(fDPSH3(0.51), fUGPSHZ3(1000), fOO9, fOOO, fOOO, fOOO, 0.0, 0.0);
	Rt.Set2(fXXZ(0.2,0.6), fdLLH(10.0,30.0), fXXZ(0.1,0.1), 1.0*DEG);
	Rmax = Rt*100;  Rmin = Rt*0.0001;  Rb.SetBit(077, 0.5);
	FBTau = .10;
}

void CSGOClbt::SetFt(int nnq)
{
	CSINSGNSS::SetFt(18);
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
	Kod *= 1 - FBXk.dd[19];
	lvOD += *(CVect3*)&FBXk.dd[21];
	dtGNSSdelay += FBXk.dd[24];
	dyawGNSS += FBXk.dd[25];
}

int CSGOClbt::Update(const CVect3 *pwm, const CVect3 *pvm, double dS, int nn, double ts, int nSteps)
{
	int res = CSINSGNSSOD::Update(pwm, pvm, dS, nn, ts, nSteps);
	if(odVelOK) {
		Hk.SetMat3(6, 0, odMphi); Hk.SetMat3(6, 3, odMvn); Hk.SetMat3(6, 18, odMkappa); Hk.SetMat3(6, 21, odMlever);  // lvOD
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
	Rt.Set2(fXYZ(0.1,0.1,0.1));
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
	if(odVelOK) {
		Hk.SetMat3(0, 0, odMphi); Hk.SetMat3(0, 3, odMvn); Hk.SetMat3(0, 15, odMkappa);
		*(CVect3*)&Zk.dd[0] = odZk;  // SINS/OD-dvn
		SetMeasFlag(07);
	}
	return 1;
}

//***************************  class CCAM & CCALLH  *********************************/
void CCAM::Init(const CVect3 &pva, const CVect3 &qt, double rp, double rv)
{
	Phi = I33;
	Pk = diag(pow(CVect3(100.0/RE,10.0,10000*UG),2));
	Xk = pva;
	Init(qt, rp, rv);
}

void CCAM::Init(const CVect3 &qt, double rp, double rv)
{
	Qt = pow(qt,2);
	Rpk = pow2(rp); Rvk=pow2(rv);
}

void CCAM::Update(double an, double ts, double Zpk, double Zvk)
{
	Xk = Phi*Xk;  Xk.j+=an*ts;	// Xkk_1=Phi*Xk+[0;a*nts;0]
	Pk = Phi*Pk*(~Phi)+Qt*ts;	// Pkk_1=Phi*Pk*(~Phi)+Qk
	Update(Zpk, Zvk);
}
	
void CCAM::Update(double Zpk, double Zvk)
{
	CVect3 PHT, Kk;
	if(!IsZero(Zpk)) {
		PHT = Pk.GetClm(0);				// PHT=Pkk_1*Hpk, where Hpk=[1,0,0]
		Kk = PHT*(1.0/(PHT.i+Rpk));		// PHT.i=dot(Hpk,PHT)
		Xk = Xk+Kk*(Zpk-Xk.i);			// Xkk_1.i=dot(Hpk,Xkk_1)
		Pk = (I33-CMat3(Kk,O31,O31,0))*Pk;  symmetry(Pk);  // vxv(Kk,Hpk)=CMat3(Kk,O31,O31,0)
	}
	if(!IsZero(Zvk)) {
		PHT = Pk.GetClm(1);				// PHT=Pkk_1*Hvk, where Hvk=[0,1,0]
		Kk = PHT*(1.0/(PHT.j+Rvk));
		Xk = Xk+Kk*(Zvk-Xk.j);
		Pk = (I33-CMat3(O31,Kk,O31,0))*Pk;
	}
}

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

//***************************  class CEarth  *********************************/
CEarth::CEarth(double a0, double f0, double g0)
{
	a = a0;	f = f0; wie = glv.wie; 
	b = (1-f)*a;
	e = sqrt(a*a-b*b)/a;	e2 = e*e;
	gn = O31;  pgn = 0;
	Update(O31);
}

void CEarth::Update(const CVect3 &pos, const CVect3 &vn, int isMemsgrade)
{
	this->pos = pos;  this->vn = vn;
	sl = sin(pos.i), cl = cos(pos.i), tl = sl/cl;
	double sq = 1-e2*sl*sl, sq2 = sqrt(sq);
	RMh = a*(1-e2)/sq/sq2+pos.k;	f_RMh = 1.0/RMh;
	RNh = a/sq2+pos.k;    clRNh = cl*RNh;  f_RNh = 1.0/RNh; f_clRNh = 1.0/clRNh;
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
		wnin = wnie + wnen;
		sl2 = sl*sl, sl4 = sl2*sl2;
		gn.k = -( glv.g0*(1+5.27094e-3*sl2+2.32718e-5*sl4)-3.086e-6*pos.k );
		gcc = pgn ? *pgn : gn;
		gcc -= (wnie+wnin)*vn;
	}
}

CVect3 CEarth::vn2dpos(const CVect3 &vn, double ts) const
{
	return CVect3(vn.j*f_RMh, vn.i*f_clRNh, vn.k)*ts;
}

//***************************  class CIMU  *********************************/
CIMU::CIMU(void)
{
	nSamples = 1;
	preFirst = onePlusPre = true; 
	phim = dvbm = wm_1 = vm_1 = O31;
	pgSens = pgSens2 = pgSensX = NULL; pKa2 = NULL; prfu = NULL;
}

void CIMU::SetRFU(const char *rfu0)
{
	for(int i=0; i<3; i++) rfu[i]=rfu0[i];
	prfu = rfu;
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

void CIMU::Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts)
{
	static double conefactors[5][4] = {				// coning coefficients
		{2./3},										// 2
		{9./20, 27./20},							// 3
		{54./105, 92./105, 214./105},				// 4
		{250./504, 525./504, 650./504, 1375./504}	// 5
		};
	int i;
	double *pcf = conefactors[nSamples-2];
	CVect3 cm(0.0), sm(0.0); wmm=O31, vmm=O31;

	psinsassert(nSamples>0 && nSamples<6);
	this->nSamples = nSamples;
	if(nSamples==1 && onePlusPre)  // one-plus-previous sample
	{
		if(preFirst) { wm_1=pwm[0]; vm_1=pvm[0]; preFirst=false; }
		cm = 1.0/12*wm_1;
		sm = 1.0/12*vm_1;
	}
	for(i=0; i<nSamples-1; i++)
	{
		cm += pcf[i]*pwm[i];
		sm += pcf[i]*pvm[i];
		wmm += pwm[i];
		vmm += pvm[i];
	}
	wm_1=pwm[i];  vm_1=pvm[i];
	wmm += pwm[i];
	vmm += pvm[i];
	phim = wmm + cm*pwm[i];
	dvbm = vmm + 1.0/2*wmm*vmm + (cm*pvm[i]+sm*pwm[i]);
	if(pgSens) {
		phim.i -= gSens.e00*dvbm.i+gSens.e01*dvbm.j+gSens.e02*dvbm.k;   // gSens.eij in (rad/s)/(m/ss)
		phim.j -= gSens.e10*dvbm.i+gSens.e11*dvbm.j+gSens.e12*dvbm.k;
		phim.k -= gSens.e20*dvbm.i+gSens.e21*dvbm.j+gSens.e22*dvbm.k;
	}
	if(pgSens2) {
		double fx2_Ts = dvbm.i*dvbm.i/ts,  fy2_Ts = dvbm.j*dvbm.j/ts,  fz2_Ts = dvbm.k*dvbm.k/ts;
		phim.i -= gSens2.e00*fx2_Ts+gSens2.e01*fy2_Ts+gSens2.e02*fz2_Ts;   // gSens2.eij in (rad/s)/(m/ss)^2
		phim.j -= gSens2.e10*fx2_Ts+gSens2.e11*fy2_Ts+gSens2.e12*fz2_Ts;
		phim.k -= gSens2.e20*fx2_Ts+gSens2.e21*fy2_Ts+gSens2.e22*fz2_Ts;
	}
	if(pgSensX) {
		double fxy_Ts = dvbm.i*dvbm.j/ts,  fyz_Ts = dvbm.j*dvbm.k/ts,  fzx_Ts = dvbm.k*dvbm.i/ts;
		phim.i -= gSensX.e00*fxy_Ts+gSensX.e01*fyz_Ts+gSensX.e02*fzx_Ts;   // gSensX.eij in (rad/s)/(m/ss)^2
		phim.j -= gSensX.e10*fxy_Ts+gSensX.e11*fyz_Ts+gSensX.e12*fzx_Ts;
		phim.k -= gSensX.e20*fxy_Ts+gSensX.e21*fyz_Ts+gSensX.e22*fzx_Ts;
	}
	if(pKa2) {
		dvbm.i -= Ka2.i*dvbm.i*dvbm.i/ts;   // Ka2.i in (m/ss)/(m/ss)^2
		dvbm.j -= Ka2.j*dvbm.j*dvbm.j/ts;
		dvbm.k -= Ka2.k*dvbm.k*dvbm.k/ts;
	}
	if(prfu) IMURFU(&phim, &dvbm, 1, prfu);
}

void IMURFU(CVect3 *pwm, int nSamples, const char *str)
{
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

//***************************  class CSINS  *********************************/
CSINS::CSINS(double &yaw0, const CVect3 &pos0, double tk0)
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

void CSINS::Init(const CQuat &qnb0, const CVect3 &vn0, const CVect3 &pos0, double tk0)
{
	tk = tk0;  ts = nts = 1.0;
	velMax = 400.0; hgtMin = -RE*0.01, hgtMax = -hgtMin; afabar = 0.1;
	qnb = qnb0;	vn = vn0, pos = pos0;
	Kg = Ka = I33; eb = db = Ka2 = O31;
	Maa = Mav = Map = Mva = Mvv = Mvp = Mpv = Mpp = O33;
	SetTauGA(CVect3(INF),CVect3(INF));
	CVect3 wib(0.0), fb=(~qnb)*CVect3(0,0,glv.g0);
	lvr = an = anbar = webbar = O31;
	isOpenloop = isMemsgrade = isNocompasseffect = isOutlever = 0;
	Cbn = I33;
	Update(&wib, &fb, 1, 1.0); imu.preFirst = 1;
	tk = tk0;  ts = nts = 1.0; qnb = qnb0;	vn = vn0, pos = pos0;
	mmwb = mmfb = CMaxMin(100);
	mvn = mvnmax = mvnmin = vn; mvni = O31; mvnt = mvnT = 0.0; mvnk = 0;
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
	if(isMemsgrade) {
		this->ts = ts;  nts = nSamples*ts;	tk += nts;
		double nts2 = nts/2;
		imu.Update(pwm, pvm, nSamples);
		imu.phim = Kg*imu.phim - eb*nts; imu.dvbm = Ka*imu.dvbm - db*nts;  // IMU calibration
		imu.dvbm.i -= Ka2.i*imu.dvbm.i*imu.dvbm.i/nts;
		imu.dvbm.j -= Ka2.j*imu.dvbm.j*imu.dvbm.j/nts;
		imu.dvbm.k -= Ka2.k*imu.dvbm.k*imu.dvbm.k/nts;
//		CVect3 vn01 = vn+an*nts2, pos01 = pos+eth.vn2dpos(vn01,nts2);
		if(!isOpenloop) eth.Update(pos,O31,1);
		wib = imu.phim/nts; fb = imu.dvbm/nts;
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
		this->ts = ts;  nts = nSamples*ts;	tk += nts;
		double nts2 = nts/2;
		imu.Update(pwm, pvm, nSamples);
		imu.phim = Kg*imu.phim - eb*nts; imu.dvbm = Ka*imu.dvbm - db*nts;  // IMU calibration
		CVect3 vn01 = vn+an*nts2, pos01 = pos+eth.vn2dpos(vn01,nts2);
		if(!isOpenloop) eth.Update(pos01, vn01);
		wib = imu.phim/nts; fb = imu.dvbm/nts;
		web = wib - Cbn*eth.wnie;  webbar = (1-afabar)*webbar + afabar*web;
//		wnb = wib - (~(qnb*rv2q(imu.phim/2)))*eth.wnin;
		wnb = wib - Cbn*eth.wnin;
		fn = qnb*fb;
		an = rv2q(-eth.wnin*nts2)*fn+eth.gcc;  anbar = (1-afabar)*anbar + afabar*an;
		CVect3 vn1 = vn + an*nts;
		pos = pos + eth.vn2dpos(vn+vn1, nts2);	vn = vn1;
		qnb = rv2q(-eth.wnin*nts)*qnb*rv2q(imu.phim);
		Cnb = q2mat(qnb); att = m2att(Cnb); Cbn = ~Cnb; vb = Cbn*vn;
	}
	psinsassert(pos.i<85.0*PI/180 && pos.i>-85.0*PI/180);
	if(vn.i>velMax) vn.i=velMax; else if(vn.i<-velMax) vn.i=-velMax;
	if(vn.j>velMax) vn.j=velMax; else if(vn.j<-velMax) vn.j=-velMax;
	if(vn.k>velMax) vn.k=velMax; else if(vn.k<-velMax) vn.k=-velMax;
	if(pos.i>89.9*DEG) pos.i=89.9*DEG; else if(pos.i<-89.9*DEG) pos.i=-89.9*DEG;
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
	mmwb.Update(norm(imu.wmm/nts));
	mmfb.Update(norm(imu.vmm/nts)+eth.gn.k);
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
	velPre = 0.0;  velMax=50.0; velMin=-3.0;  afa = 1.0;
	tk = tk0;
}

void CDR::Update(const CVect3 &wm, double dS, double ts)
{
	tk += ts;
	double vel = dS*Kod/ts;
	if(vel>velMax||vel<velMin) vel=velPre;  // if abnormal, keep last velocity
	CVect3 vnk = Cnb*(Cbo*CVect3(0,vel,0));  velPre=vel;
	eth.Update(pos, vnk);
	vn = (1-afa)*vn + afa*vnk;  // AR(1) vel filter
	pos = pos + eth.vn2dpos(vnk, ts); 
	qnb = rv2q(-eth.wnin*ts)*qnb*rv2q(wm);
	Cnb = q2mat(qnb); att = m2att(Cnb);
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

void CAVPInterp::Init(const CSINS &sins, double ts, BOOL islever)
{
	if(islever)
		Init(sins.att, sins.vnL, sins.posL, ts);
	else
		Init(sins.att, sins.vn, sins.pos, ts);
}

void CAVPInterp::Init(const CVect3 &att0, const CVect3 &vn0, const CVect3 &pos0, double ts)
{
	this->ts = ts;
	ipush = 0;
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
	if(++ipush>=AVPINUM) ipush = 0;
	atti[ipush] = attk; vni[ipush] = vnk; posi[ipush] = posk;
}

int CAVPInterp::Interp(double tpast, int avp)
{
	int res=1, k, k1, k2;
	if(tpast<-AVPINUM*ts) tpast=-AVPINUM*ts; else if(tpast>0) tpast=0;
//	if(tpast<-AVPINUM*ts||tpast>0) return (res=0);
	k = (int)(-tpast/ts);
	if((k2=ipush-k)<0) k2 += AVPINUM;
	if((k1=k2-1)<0)  k1 += AVPINUM;
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
	SetTau(tau);
	qnb = qnb0;
	Cnb = q2mat(qnb);
	exyzInt = O31;  ebMax = One31*glv.dps*5;
	tk = 0.0;
}

void CMahony::SetTau(double tau)
{
	double beta = 2.146/tau;
	Kp = 2.0*beta, Ki = beta*beta;
}

void CMahony::Update(const CVect3 &wm, const CVect3 &vm, double ts, const CVect3 &mag)
{
	double nm;
	CVect3 acc0, mag0, exyz, bxyz, wxyz;

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
	Rt.Set2(100.0*glv.mg/sts,100.0*glv.mg/sts, 1.0*DEG/sts);
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

#include "io.h"
char* time2fname(void)
{
	static char PSINSfname[32];
	time_t tt;  time(&tt);
	tm *Time = localtime(&tt);
	strftime(PSINSfname, 32, "PSINS%Y%m%d_%H%M%S.bin", Time);
	return PSINSfname;
}

char CFileRdWt::dirIn[256] = {0}, CFileRdWt::dirOut[256] = {0};

void CFileRdWt::DirI(const char *dirI)  // set dirIN
{
	int len = strlen(dirI);
	memcpy(dirIn, dirI, len);
	if(dirIn[len-1]!='\\') { dirIn[len]='\\'; dirIn[len+1]='\0'; }
	if(_access(dirIn,0)==-1) dirIn[0]='\0';
}

void CFileRdWt::DirO(const char *dirO)  // set dirOUT
{
	int len = strlen(dirO);
	memcpy(dirOut, dirO, len);
	if(dirOut[len-1]!='\\') { dirOut[len]='\\'; dirOut[len+1]='\0'; }
	if(_access(dirOut,0)==-1) dirOut[0]='\0';
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
		fpos_t pos;
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
		fsetpos(rwf, &pos);
		this->columns = columns;
		for(int i=0; i<columns; i++)
		{ sstr[4*i+0]='%', sstr[4*i+1]='l', sstr[4*i+2]='f', sstr[4*i+3]=' ', sstr[4*i+4]='\0'; } 
	}
	else
	{
		rwf = 0;
	}
	linek = 0;
	totsize = 0;
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
				if(*pc==','||*pc==';'||*pc==':'||*pc=='\t:') *pc=' ';
				else if(*pc=='\0') break;
			}
		}
		if(columns<10)
			sscanf(line, sstr,
				&buff[ 0], &buff[ 1], &buff[ 2], &buff[ 3], &buff[ 4], &buff[ 5], &buff[ 6], &buff[ 7], &buff[ 8], &buff[ 9]
				); 
		else if(columns<20)
			sscanf(line, sstr,
				&buff[ 0], &buff[ 1], &buff[ 2], &buff[ 3], &buff[ 4], &buff[ 5], &buff[ 6], &buff[ 7], &buff[ 8], &buff[ 9],
				&buff[10], &buff[11], &buff[12], &buff[13], &buff[14], &buff[15], &buff[16], &buff[17], &buff[18], &buff[19]
				); 
		else if(columns<40)
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
	for(int i=0; i<-columns; i++) buff[i]=buff32[i];
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

void CFileRdWt::bwseek(int lines, int mod)  // double-bin file backward-seek lines
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

BOOL CFileRdWt::waitfor(int columnk, double val, double eps)
{
	double wf=buff[columnk]-val;
	while(-eps<wf && wf<eps) {
		load(1);
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
		*this<<mm.mm[i].maxRes<<mm.mm[i].minRes<<mm.mm[i].maxpreRes<<mm.mm[i].minpreRes<<(double)mm.mm[i].flag;
	return *this;
}

CFileRdWt& CFileRdWt::operator<<(const CAligni0 &aln)
{
	return *this<<q2att(aln.qnb)<<aln.vib0<<aln.Pi02<<aln.tk;
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
	return *this<<dr.att<<dr.vn<<dr.pos<<dr.tk;
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

#endif // PSINS_IO_FILE

//******************************  CRMemory *********************************/
#ifdef PSINS_RMEMORY
#pragma message("  PSINS_RMEMORY")

CRMemory::CRMemory(void)
{
	pMemStart0 = NULL;
}

CRMemory::CRMemory(long recordNum, int recordLen0)
{
	BYTE *pb = (BYTE*)malloc(recordNum*recordLen0);
	*this = CRMemory(pb, recordNum*recordLen0, recordLen0);
	pMemStart0 = pMemStart;
}

CRMemory::CRMemory(BYTE *pMem, long memLen0, int recordLen0)
{
	psinsassert(recordLen0<=MAX_RECORD_BYTES);
	pMemStart0 = NULL;
	pMemStart = pMemPush = pMemPop = pMem;
	pMemEnd = pMemStart + memLen0;
	pushLen = popLen = recordLen = recordLen0;
	memLen = memLen0;
	dataLen = 0;
}

CRMemory::~CRMemory()
{
	if(pMemStart0) { free(pMemStart0); pMemStart0 = NULL; } 
}

BYTE CRMemory::pop(BYTE *p)
{
	if(dataLen==0) return 0;
	popLen = recordLen==0 ? *pMemPop : recordLen;
	if(p==(BYTE*)NULL) p = popBuf;
	BYTE i;
	for(i=0; i<popLen; i++,dataLen--)
	{
		*p++ = *pMemPop++;
		if(pMemPop>=pMemEnd)  pMemPop = pMemStart;
	}
	return i;
}

BYTE* CRMemory::get(int iframe)
{
	return &pMemStart[popLen*iframe];
}

BYTE* CRMemory::set(int iframe, const BYTE *p)
{
	BYTE *p0 = &pMemStart[pushLen*iframe];
	BYTE i;
	for(i=0; i<pushLen; i++)
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
	for(BYTE i=0; i<pushLen; i++,dataLen++)
	{
		*pMemPush++ = *p++;
		if(pMemPush>=pMemEnd)  pMemPush = pMemStart;
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
	Delete(pmem);
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
	if(pmem->pMemStart0) Delete(pmem);
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
CVect3 Alignsb(const CVect3 wmm, const CVect3 vmm, double latitude)
{
	double T11, T12, T13, T21, T22, T23, T31, T32, T33;
	double cl = cos(latitude), tl = tan(latitude), nn;
	CVect3 wbib = wmm / norm(wmm),  fb = vmm / norm(vmm);
	T31 = fb.i,				 T32 = fb.j,			 	T33 = fb.k;
	T21 = wbib.i/cl-T31*tl,	 T22 = wbib.j/cl-T32*tl,	T23 = wbib.k/cl-T33*tl;		nn = sqrt(T21*T21+T22*T22+T23*T23);  T21 /= nn, T22 /= nn, T23 /= nn;
	T11 = T22*T33-T23*T32,	 T12 = T23*T31-T21*T33,		T13 = T21*T32-T22*T31;		nn = sqrt(T11*T11+T12*T12+T13*T13);  T11 /= nn, T12 /= nn, T13 /= nn;
	CMat3 Cnb(T11, T12, T13, T21, T22, T23, T31, T32, T33);
	return m2att(Cnb);
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
	imu.Update(pwm, pvm, nSamples);
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
	Pmin.Set2(fXXZU(1.0,10.0, SEC), fXXX(0.001), fXXX(0.01), fDPH3(0.001), fUG3(1.0));
	Pk.SetDiag2(fXXZU(1.10,10.0, DEG), fXXX(1.0), fXXX(100.0), fDPH3(0.05), fUG3(100.0));
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
	Rmax = Rt * 100;  Rmin = Rt*0.01;  Rb = 0.9;
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
	Delete(pMem);
	pMem = new CRMemory((int)(T1/ts+10), 6*sizeof(double));
}

CAlignsv::~CAlignsv()
{
	Delete(pMem);
}

int CAlignsv::Update(const CVect3 *pwm, const CVect3 *pvm, int nSteps)
{
	double wvm[6];
	*(CVect3*)&wvm[0] = *pwm, *(CVect3*)&wvm[3] = *pvm;
	pMem->push((const unsigned char*)wvm);
	if(t<T1) {  // align_i0
		alni0.Update((CVect3*)&wvm[0], (CVect3*)&wvm[3], 1, ts);
		qnb = alni0.qnb;
		tk += ts;  t += ts;
	}
	else {      // align_kf
		if(!alnkfinit) {
			alnkfinit = 1;  tk = 0.0;
			CAlignkf::Init(CSINS(alni0.qnb0, O31, alni0.pos0));
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
	// states: 0-2:phi,3-5:dvn,6-8:mu,9-11:eb,12-14:db,15-17:lv,18:dT
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
	Ft.SetMat3(6,3,O33), Ft.SetMat3(6,6,O33);		// state6-8:dpos->mu
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
	sins.qnb -= *(CVect3*)&FBXk.dd[0];  sins.vn -= *(CVect3*)&FBXk.dd[ 3];  mu -= *(CVect3*)&FBXk.dd[6];
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
			*(CVect3*)&Zk.dd[3] = qq2phi(a2qua(avpi.att)*rv2q(mu), a2qua(attMINS));
			SetMeasFlag(00070);
		}
	}
}

//***************************  class CUartPP  *********************************/
#ifdef PSINS_CONSOLE_UART

#pragma message("  PSINS_CONSOLE_UART")

CUartPP::CUartPP(int frameLen0, unsigned short head0)
{
	popIdx = 0;
	pushIdx = 1;
	unsigned char *p = (unsigned char*)&head0;
	head[0] = p[1], head[1] = p[0];
//	*(unsigned short*)head = head0;
	frameLen = frameLen0;
	csflag = 1, cs0 = 4, cs1 = frameLen-1, css = 2;
	overflow = getframe = 0;
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

//***************************  class CConUart  *********************************/
CUartPP *pUart;

CConUart::CConUart(void)
{
	hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
}

void CConUart::Init(int COMi, int BaudRate, int frameLen, unsigned short head)
{
	uart = CUartPP(frameLen, head);  uart.csflag = 0;  pUart = &uart;
	ps = (PSINSBoard*)&uart.popbuf[0];
	hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTitle("PSINS开发板上位机测试软件(by YGM@NWPU)");
	if(!m_SerialPort.InitPort(GetConsoleWindow(), comi=COMi, BaudRate))
	{
		printf("\nCOM%d open failed!\n\n", COMi);  exit(0);
	}
	m_SerialPort.StartMonitoring();
//	atexit(fraw.~CFileRdWt());
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
	CVect3 vtmp;
	double *p=&vtmp.i; const double *pmu=&mu.i, *psigma=&sigma.i;
	for(int i=0; i<3; i++,p++,pmu++,psigma++) *p=randn(*pmu,*psigma);
	return vtmp;
}

CVect randn(const CVect &mu, const CVect &sigma)
{
	CVect vtmp(mu.row,mu.clm);
	double *p=&vtmp.dd[0]; const double *pmu=&mu.dd[0], *psigma=&sigma.dd[0];
	for(int i=0; i<vtmp.rc; i++,p++,pmu++,psigma++) *p=randn(*pmu,*psigma);
	return vtmp;
}

CMat3 randn(const CMat3 &mu, const CMat3 &sigma)
{
	CMat3 mtmp;
	double *p=&mtmp.e00; const double *pmu=&mu.e00, *psigma=&sigma.e00;
	for(int i=0; i<9; i++,p++,pmu++,psigma++) *p=randn(*pmu,*psigma);
	return mtmp;
}

