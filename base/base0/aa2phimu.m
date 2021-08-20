function [phi, mu0, phi0] = aa2phimu(att1, att0, mu, isfig)
% Calculate platform misalignment angles(should be small) between att1 and
% att0. This function is suitable for batch processing.
%
% Prototype: phi = aa2phi(att1, att0)
% Inputs: att1 - Euler angles att1=[pitch1;roll1;yaw1]
%         att0 - reference Euler angles att0=[pitch0;roll0;yaw0]
% Output: phi - platform misalignment angles from computed nav-frame to
%               ideal nav-frame, phi=[phiE;phiN;phiU]
%
% See also  aa2phi, aa2mu, aa2dkg.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/10/2020
global glv
    [z, i1, i0] = avpcmp(att1, att0, 'phi');
    n = length(z); I33 = eye(3);
    Z = reshape(z(:,1:3)',3*n,1);  H = zeros(3*n,6);  % Z = Cnbs*Cnbm' = phi - Cnb*mu
    for k=1:n
        H(3*k-2:3*k,:) = [I33, a2mat(att0(i0(k),1:3)')];
    end
    X = lscov(H, Z);  phi0=X(1:3); mu0=X(4:6);
    if nargin==3, mu0=mu; end
    Cbb = a2mat(mu0)';
    for k=1:n
        att0(i0(k),1:3) = m2att(a2mat(att0(i0(k),1:3)')*Cbb)';  % aaddmu
    end
    phi = avpcmp(att1, att0(i0,:), 'phi');
    if ~exist('isfig','var'), isfig=0; end
    if isfig==1
        myfig;
        subplot(211), plot(z(:,end), z(:,1:3)/glv.min); xygo('phi');
        subplot(212), plot(phi(:,end), phi(:,1:3)/glv.min); xygo('phi');
        title(sprintf('mu=%.2f, %.2f, %.2f (arcmin)', mu0(1)/glv.min, mu0(2)/glv.min, mu0(3)/glv.min));
    end 
    