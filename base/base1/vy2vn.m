function vn = vy2vn(vel, yaw)
% Convert vel&yaw to vn.
%
% Prototype: vn = vy2vn(vel, yaw)
% Inputs: vel - velocity norm
%         yaw - yaw, [-pi,pi]
% Output: vn - velocity vector in n-frame
%
% See also  vn2vb, vb2vn, vn2att.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/02/2021
    if length(vel)<3
        if nargin<2, yaw=vel(2); end
        vn = [vel*sin(-yaw), vel*cos(yaw), vel*0]';
        return;
    end
    if nargin<2, yaw=vel(:,2); end
    vn = [vel.*sin(-yaw), vel.*cos(yaw), vel*0];