function [att, phi] = fb2atti(fb, att0)
% Using sepecific force vector to determine attitude.
%
% Prototype: [att, phi] = fb2atti(fb, att0)
% Inputs: fb - sepecific force in static base
%         att0 - coarse attitude, yaw, quaternion or DCM
% Outputs: att - fine quaternion, Euler angles or DCM form as input att0
%          phi - residual error
%
% Example:
%   [att, phi] = fb2atti([0;0.01;9.8], 30*glv.deg)
%
% See also  sv2atti, dv2atti, mv2atti, alignsb.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/04/2024
    [m,n] = size(att0); mn=m*n;
    if mn==1, att0=[0;0;att0]; mn=3; end
    [qnb,att,Cnb] = attsyn(att0);
    fb = fb/norm(fb);
    phi = Cnb*fb;  phi=[phi(2);-phi(1);0];
    qnb = qdelphi(qnb,phi);
    if mn==3, att=q2att(qnb);
    elseif mn==4, att=qnb;
    elseif mn==9, att=q2mat(qnb);
    end
    