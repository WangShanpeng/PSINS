function [att1, ins] = aligni0vn(imu, pos, t1)
% SINS initial align based on inertial-frame & vn-meas method.
%
% Prototype: att0 = aligni0(imu, pos, t1)
% Inputs: imu - IMU data
%         pos - position
%         t1 - inertial-frame align time
% Output: att1 - attitude align result
%         ins - SINS struct
%
% Example:
%     glvs;
%     [imu, avp0, ts] = imufile('lasergyro.imu');
%     att = aligni0vn(imu(1:300/ts,:), avp0(7:9)', 180);
%
% See also  aligni0, alignvn.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/11/2020
global glv
    ts = imu(2,end)-imu(1,end);
    if nargin<3, t1 = 30; end
    if(length(pos)>4) pos=pos(4:6); end
    imu1 = imu(1:fix(min(imu(end,end)-imu(1,end),t1)/ts),:);
    [att0, res0] = aligni0(imu1, pos, ts);  % coarse align
    [att0, iatt] = attrvs(imu1, att0, pos);  % reverse attitude update
	phi = [1.1; 1.1; 10]*glv.deg;
	imuerr = imuerrset(0.002, 20, 0.001, 10);
	wvn = [0.01; 0.01; 0.01]*10;
	[att1, attk] = alignvn(imu, a2qua(att0), pos, phi, imuerr, wvn);  % fine align
    insplot(attk,'a');
    subplot(211), plot(res0.attk(:,end), res0.attk(:,1:2)/glv.deg, 'm');
    subplot(212), plot(res0.attk(:,end), res0.attk(:,3)/glv.deg, 'm');
    subplot(212), plot(iatt(:,end), iatt(:,3)/glv.deg, ':r', 'linewidth',2); legend('KF fine yaw', 'i0 coarse yaw', 'reverse yaw');
    for k=1:2  % reverse & fine iteration
        [att0, iatt] = attrvs(imu, attk(end,1:3)', pos);
        subplot(212), plot(iatt(:,end), iatt(:,3)/glv.deg, ':r', 'linewidth',2);
        phi = [0.1; 0.1; 1]/k*glv.deg;
        [att1, attk] = alignvn(imu, a2qua(att0), pos, phi, imuerr, wvn);  close(clf);
        subplot(211), plot(attk(:,end), attk(:,1:2)/glv.deg, 'linewidth',k+1);
        subplot(212), plot(attk(:,end), attk(:,3)/glv.deg, 'linewidth',k+1);
    end
    if nargout==2, ins=insinit([att1;pos], ts); end

