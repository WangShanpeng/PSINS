function [dKg, dphi] = aa2dkg(att0, att1, t0, t1)
% Calculate gyro calibration error between att1 and att0, from t0 to t1. 
%
% Prototype: dkg = aa2dkg(att0, att1, t0, t1)
% Inputs: att1 - Euler angle array att1
%         att0 - reference Euler angle array att0
%         t0,t1 - time from t0 to t1
% Output: dKg - gyro calibration error. 
%
% Example:
%    [dKg, xyz] = aa2dkg(att0, att1, 100, 120);
%    imu1 = imuclbt(imu, eye(3)-dKg, [0;0;0]);
%
% See also  aa2phi, aa2mu, aa2phimu.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/05/2021
    if ~exist('t0', 'var'), t0 = max(att0(1,end),att1(1,end)); end
    if ~exist('t1', 'var'), t1 = min(att0(end,end),att1(end,end)); end
    att0 = datacut(att0(:,[1:3,end]), t0, t1);
    att1 = datacut(att1(:,[1:3,end]), t0, t1);
    wts = aa2mu(att0(2:end,1:3), att0(1:end-1,1:3)); % mu=wbib*ts
    Cx = zeros(3); Cy = zeros(3); Cz = zeros(3); 
    for k=1:length(att0)-1
        C = a2mat(att0(k,1:3)');
        Cx = Cx - wts(k,1)*C; Cy = Cy - wts(k,2)*C; Cz = Cz - wts(k,3)*C;
    end
    [~, xyz] = max(abs(sum(wts)));
    phi = aa2phimu(att1, att0, [0;0;0], 1);
    dphi = phi(end,1:3)'-phi(1,1:3)';
%     dphi = aa2phi(att1(end,1:3)', att0(end,1:3)') - aa2phi(att1(1,1:3)', att0(1,1:3)');
    dKg = zeros(3);
    switch(xyz)
        case 1,  dKg(:,1) = - Cx^-1 * dphi;
        case 2,  dKg(:,2) = - Cy^-1 * dphi;
        case 3,  dKg(:,3) = - Cz^-1 * dphi;
    end
    
    