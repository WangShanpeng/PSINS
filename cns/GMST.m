function [theta, h, T] = GMST(jd, s)
% GMST (Greenwich Mean Sidereal Time) calculation
% Ref. https://blog.csdn.net/zhuimengshizhe87/article/details/26702997
% Verified by. http://dc.zah.uni-heidelberg.de/apfs/times/q/form
%
% Prototype: [theta, h, T] = GMST(jd, s)
% Inputs: jd - UTC Julian date
%         s - UT1 seconds within a day, using UT1 to predict the Earth rotation
% Outputs: theta - sidereal angle, in rad
%          h - sidereal time within a day, in hour
%          T - time diff in century
%
% Examples:
%   [theta, h, T] = GMST(JD(2000,1,15), hms2s([10,30,23]));  s2hms(h*3600);
%
% See also  GAST, JD, MJD

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/11/2021
    if nargin<2, s = 0; end  % s in second
    Td = (jd-2451545)/36525;  % date diff in J-Century, from JD(2000,1,1)+0.5;
    h = 6.697374558 + 2400.051336*Td + 0.000025862*Td*Td + s/3600*1.00273790935;  % in hour
    h = mod(h, 24);
    theta = h*15*pi/180;  % in rad
    T = Td + s/86400/36525;  % 11/06/2022
    
    