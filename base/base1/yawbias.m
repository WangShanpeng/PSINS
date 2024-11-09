function yaw = yawbias(yaw, bias)
% Yaw bias setting.
%
% Prototype: yaw = yawbias(yaw, bias)
% Inputs: yaw - input yaw angles, in rad
%         bias - yaw bias with (-pi, pi)
% Output: yaw - output yaw angles
%
% See also  yawadd, avpcvt, att2c.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 25/10/2024
    if size(yaw,2)==1
        yaw = yaw+bias;
        idx = yaw>=pi;
        yaw(idx) = yaw(idx)-2*pi;
        idx = yaw<-pi;
        yaw(idx) = yaw(idx)+2*pi;
    elseif size(yaw,2)<=3  % [*, yaw, t]
        yaw(:,end-1) = yawbias(yaw(:,end-1), bias);
        return;
    else  % [*,*,yaw,***]
        yaw(:,3) = yawbias(yaw(:,3), bias);
        return;
    end

