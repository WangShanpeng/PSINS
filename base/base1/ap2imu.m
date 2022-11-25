function  [imu, avp0, avp] = ap2imu(ap, ts)
% Simulate SIMU sensor outputs from attitude & position profile.
%
% Prototype: [imu, avp0, avp] = ap2imu(ap, ts)
% Inputs: ap = [att, pos, t]
%         ts - sampling time
% Outputs: imu = [wm,vm,t]
%          avp0 = init [att,vn,pos]
%          avp = [att,vn,pos,t] array
%
% Example:
%   ts = 0.01; t = (1:ts:10)';
%   pitch = 3; roll = 5; yaw = 1; f1 = 2*pi*0.2; f2 = 2*pi*0.5; f3 = 2*pi*0.1; 
%   ap = [[pitch*sin(f1*t), roll*sin(f2*t), yaw*sin(f3*t)]*glv.deg, repmat(glv.pos0',length(t),1), t];
%   [imu, avp0] = ap2imu(ap, 0.001);
%   imuplot(imu);
%
% See also  ap2avp, avp2imu, att2c, pos2c.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/03/2020
    if nargin<2, ts = diff(ap(1:2,end)); end
    avp = ap2avp(ap, ts);
    [imu, avp0] = avp2imu(avp);
    