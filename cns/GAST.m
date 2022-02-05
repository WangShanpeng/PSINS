function [theta, h, T, TT, eps, dpsi, deps] = GAST(jd, s, dTT)
% GAST (Greenwich Apparent Sidereal Time) calculation
% Ref. https://blog.csdn.net/zhuimengshizhe87/article/details/26702997
% Verified by. http://dc.zah.uni-heidelberg.de/apfs/times/q/form
% https://www.nist.gov/pml/time-and-frequency-division/time-realization/leap-seconds
% https://datacenter.iers.org/singlePlot.php?plotname=BulletinA_LatestVersion-UT1-UTC&id=6     UT1-UTC
% https://www.stjarnhimlen.se/comp/time.html
% http://www.iausofa.org/
%
% Prototype: [theta, h, T, TT, eps, dpsi, deps] = GAST(jd, s, dTT)
% Inputs: jd - UTC Julian date
%         s - UT1 seconds within a day, using UT1 to predict the Earth rotation
%         dTT - = TT-UTC, using TDB to predict the precession
% Outputs: theta - sidereal angle, in rad
%          h - sidereal time within a day, in hour
%          T,TT - time diff in century, (TT for TDB century)
%
% Examples:
%   [theta, h, T, TT] = GAST(JD(2000,1,15), hms2s([10,30,23]));  s2hms(h*3600);
%   [theta, h, T, TT] = GAST(JD(2028,10,04), hms2s([10,30,23]));  s2hms(h*3600);
%
% See also  GMST, JD, MJD, Nutmat, cnscie.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/11/2021
global glv
    if nargin<3, dTT = 69.184; end
    if nargin<2, s = 0; end  % UT1 in second
	[THETA, h, T] = GMST(jd, s);
    % the following same as Nutmat
    TT = T + dTT/86400/36525; % TDB to predict the precession, may has small error 'UT1-UTC'
    [eps, dpsi, deps] = Equinox(TT);
	theta = THETA + dpsi * cos(eps+deps);
    h = h + (theta-THETA)/(15*glv.dph)/3600;
    return;
    
% test    
figure,
err = [];
for y=1970:2030,  err(y-1969,:) = [y, GAST(JD(y,1,1))-GMST(JD(y,1,1))];  end;
plot(err(:,1),err(:,2)/glv.sec,'-*'); xygo('t / year', 'GAST-GMST / (\prime\prime)');

err = zeros(10000,2);
for k=1:10000, err(k,:) = [y, GAST(JD(2000,1,1)+k)-GMST(JD(2000,1,1)+k)]; end
myfig, plot(2000+(0:9999)/365, err(:,2)/glv.sec); xygo('t / day', 'nutation / \prime\prime');

