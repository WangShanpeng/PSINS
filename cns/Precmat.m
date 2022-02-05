function [CP, zeta,theta,z] = Precmat(T)
% Luni-solar precession rotation matrix.
%
% Prototype: [CP, zeta,theta,z] = Precmat(T)
% Input: T - TDB time diff in century, from JD(2000,1,1)+0.5;
% Outputs: CP - precession rotation matrix
%          zeta,theta,z - angles in arcsec
%
% See also  Nutmat, GAST, GMST, JD, cnscie, Polmat.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/11/2021
global glv
	T2 = T^2; T3 = T^3;
    zeta  = (2306.2181*T+0.301880*T2+0.017998*T3)*glv.sec;
    theta = (2004.3109*T-0.426650*T2-0.041833*T3)*glv.sec;
    z     = (2306.2181*T+1.094687*T2+0.018203*T3)*glv.sec;
    CP = rxyz(z,'z') * rxyz(-theta,'y') * rxyz(zeta,'z');
    return;
    
% Example
phi = [];
for k=1:100, phi(k,:) = m2att(Precmat((k-1)/100))'; end
myfig, plot(2000:2099,phi/glv.deg); xygo('t / year', 'precession / \circ');

% verified by SOFA
a = []; kk=1;
dt = JD([2000,1,1])+0.5-2451545;
for k=-50:49
    T = (dt+k*365.25)/36525;
    a(kk,:) = [2000+k, m2att(Precmat(T))']; kk=kk+1;
end
sofapn = load('sofapn.txt');
figure, plot(sofapn(:,1), [sofapn(:,2:4)-a(:,2:4)]/glv.sec); xygo('t / year', 'precession error / (\prime\prime)');
