function [CN, eps,deps,dpsi] = Nutmat(T)
% Luni-solar nutation rotation matrix (IAU 2000)
% https://wenku.baidu.com/view/d0739f2d915f804d2b16c1b2.html
%
% Prototype: [CN, eps0,deps,dpsi] = Nutmat(T)
% Input: T - TDB time diff in century, from JD(2000,1,1)+0.5;
% Outputs: CN - nutation rotation matrix
%          eps0,deps,dpsi - angles in arcsec
%
% See also  Precmat, GAST, GMST, cnscie, Polmat.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/11/2021
    [eps, dpsi, deps] = Equinox(T);
    CN = rxyz(eps+deps,'x') * rxyz(dpsi,'z') * rxyz(-eps,'x');
    return
    
% Example
phi = zeros(10000,3);
for k=1:10000, phi(k,:) = m2att(Nutmat((k-1)/10000))'; end
myfig, plot(2000+(0:9999)/100, phi/glv.sec); xygo('t / year', 'nutation / \prime\prime');
% verified by SOFA
a = []; kk=1;
dt = JD([2000,1,1])+0.5-2451545;
for k=-50:49
    T = (dt+k*365.25)/36525;
    a(kk,:) = [2000+k, m2att(Nutmat(T))']; kk=kk+1;
end
sofapn = load('sofapn.txt');
figure, plot(sofapn(:,1), [sofapn(:,5:7)-a(:,2:4)]/glv.sec); xygo('t / year', 'nutation error / (\prime\prime)');
figure, plot(sofapn(:,5)/glv.sec,sofapn(:,6)/glv.sec); xygo('t / year', 'nutation error / (\prime\prime)');
