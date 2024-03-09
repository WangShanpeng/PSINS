function Mt = wftau(w, f, tau)
% Ref. 'Estimation and compensation for dynamic bending error parameters
%       of mechanical dithered RLG sensitive axis'
%
% Prototype: Mt = wftau(w, f, tau)
% Inputs:  w - gyro angular rate
%          f - acc specific force
%          tau - dynamic bending error parameters (in arc/g)
% Outputs: Mt - dynamic bending error
%
% Example
%         ts = diff(imu(1:2,end));  tau=randn(9,1)*10;  imu1=imu;
%         for k=1:length(imu) imu1(k,1:3)=imu(k,1:3)+ts*wftau(imu(k,1:3)/ts,imu(k,4:6)/ts,tau)'; end

% See also  cnscl.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 04/07/2023
global glv
    M1 = [ w(3)*f(3)+w(2)*f(2)  -w(2)*f(1)           -w(3)*f(1)];
    M2 = [-w(1)*f(2)             w(3)*f(3)+w(1)*f(1) -w(3)*f(2)];
    M3 = [-w(1)*f(3)            -w(2)*f(3)            w(2)*f(2)+w(1)*f(1)];
    O13 = [0 0 0];
    Mt = [M1 O13 O13; O13 M2 O13; O13 O13 M3];
    if nargin>2
        Mt = Mt*tau*(glv.sec/glv.g0);  % in rad/s
    end
    