function [imu, avp0] = imuoppl(imu, avp0, href)
% Transfer high-grade SIMU (with low inspure pos errors, especially in static base)
% to oppsitive latitude, i.e. from the northern hemisphere to south, or vise versa.
%
% Prototype: imu = imuoppl(imu, avp0)
% Inputs: imu - gyro & acc incremental inputs
%         avp0 - initial avp0=[att0,vn0,pos0]
%         href - height reference type string, see inspure
% Outputs: imu - gyro & acc incremental outputs with oppsitive latitude.
%
% Example:
%     glvs;
%     [imu, avp0, ts] = imufile('lasergyro.imu');  imuplot(imu);
%     [~, res] = aligni0(imu(1:300/ts,:), avp0);  avp0(1:3)=res.att0;
%     [imu1, avp1] = imuoppl(imu, avp0); imuplot(imu1);
%     avp = inspure(imu1, avp1, 'f');
%
% See also  imustatic, inspure, ap2imu.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/01/2021
    if nargin<3, href = 'H'; end
    avp = inspure(imu, avp0, href);
    p = polyfit(avp(:,end),avp(:,7),6);
    avp(:,7) = -avp0(7) + (avp(:,7)-polyval(p,avp(:,end)));
    p = polyfit(avp(:,end),avp(:,8),6);
    avp(:,8) = avp0(8) + (avp(:,8)-polyval(p,avp(:,end)));  % insplot(avp);
    [imu, avp0] = ap2imu(avp(:,[1:3,7:10]), diff(imu(1:2,end)));

