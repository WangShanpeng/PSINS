function yaw = yawadd(yaw, dyaw)
% Yaw angles to add some dyaw.
%
% Prototype: yaw = yawadd(yaw, dyaw)
% Inputs: yaw - input yaw angles, in rad
%         dyaw - yaw to be added
% Output: yaw - output yaw angles within [-pi, pi]
%
% Example
%    figure,  plot(yawadd((1:0.1:10)',1))
%
% See also  yawcvt, att2c.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 24/08/2022
    yaw = yaw + dyaw;
    yaw = atan2(sin(yaw),cos(yaw));