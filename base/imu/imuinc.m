function [ang, wb, vel, fb] = imuinc(imu, t0, t1)
% Calculate angle/velocity increament from time t0 to t1.
%
% Prototype: [ang, wb] = anginc(imu, t0, t1)
% Inputs: imu - SIMU data array
%         t0 - start time
%         t1 - end time
% Outputs: ang, wb - angle increament & mean angular rate
%:         vel, fb - apparent velocity increament & mean force
%
% See also  imudot, imurot.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 18/01/2021
    imu = datacut(imu, t0, t1);
    inc = sum(imu(:,1:6))';
    ang = inc(1:3); wb = ang/(t1-t0);
	vel = inc(4:6); fb = vel/(t1-t0);