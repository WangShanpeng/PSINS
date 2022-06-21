function qnb = setyaw(qnb, yaw)
% Set attitude with specific yaw.
%
% Prototype: qnb = setyaw(qnb, yaw)
% Inputs: qnb - attitude quaternion/Euler angles/DCM
%         yaw - specific yaw to be set
% Output: qnb - attitude with yaw 
%
% See also  a2qua, a2mat.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/06/2022
    [m,n] = size(qnb);  mn = m*n;
    if mn==3, qnb(3)=yaw;  % Euler angles
    elseif mn==4, att = q2att(qnb); att(3)=yaw; qnb = a2qua(att);  % quaternion
    elseif mn==9, att = m2att(qnb); att(3)=yaw; qnb = a2mat(att);  % DCM
    end
    