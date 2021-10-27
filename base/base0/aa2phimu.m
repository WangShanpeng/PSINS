function [phi, mu0, phi0] = aa2phimu(att1, att0, mu, isfig)
% Calculate platform misalignment angles & installation error angles (should be small) 
% between att1 and att0.
%
% Prototype: [phi, mu0, phi0] = aa2phimu(att1, att0, mu, isfig)
% Inputs: att1 - Euler angles att1=[pitch1;roll1;yaw1]
%         att0 - reference Euler angles att0=[pitch0;roll0;yaw0]
%         mu - initial installation error angles mu=[mux;muy;muz]
%         isfig - figure flag
% Output: phi - platform misalignment angles from computed nav-frame to
%               ideal nav-frame, phi=[phiE;phiN;phiU]
%         mu0,phi0 - additional estimated mu & estimated phi
%
% Example:
%     att0 = [[0 0 0; 0 0 0; 0 0 90; 0 0 90; 45 0 90; 45 0 90]*glv.deg, [1; 2; 3; 4; 5; 6]];
%     phi = [10; 20; 30]*glv.min; mu = [1; 2; 3]*glv.deg; att1=att0;
%     for k=1:size(att0,1), att1(k,1:3) = m2att(rv2m(-phi)*a2mat(att0(k,1:3)')*rv2m(-mu));  end
%     [phi, mu0, phi0] = aa2phimu(att1, att0, 0*[1;2;3]*glv.deg, 1);
%
% See also  aa2phi, aa2mu, aa2dkg.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/10/2020
global glv
    if nargin<3, mu=0; end
    if norm(mu)>0
        Cbb = a2mat(mu)';
        for k=1:size(att0,1)
            att0(k,1:3) = m2att(a2mat(att0(k,1:3)')*Cbb)';  % add-dmu
        end
    end
    %%
    [z, i1, i0] = avpcmp(att1, att0, 'phi');
    n = length(z); I33 = eye(3);
    Z = reshape(z(:,1:3)',3*n,1);  H = zeros(3*n,6);  % Z = Cnbs*Cnbm' = phi - Cnb*mu
    for k=1:n
        H(3*k-2:3*k,:) = [I33, a2mat(att0(i0(k),1:3)')];
    end
    X = lscov(H, Z);  phi0=X(1:3); mu0=X(4:6);
    %%
    Cbb = a2mat(mu0)';
    for k=1:n
        att0(i0(k),1:3) = m2att(a2mat(att0(i0(k),1:3)')*Cbb)';  % aad-dmu0
    end
    phi = avpcmp(att1, att0(i0,:), 'phi');
    %%
    if ~exist('isfig','var'), isfig=0; end
    if isfig==1
        myfig;
        subplot(311), plot(att0(:,end), att0(:,1:3)/glv.deg, 'm'); xygo('att'); plot(att1(:,end), att1(:,1:3)/glv.deg);
        subplot(312), plot(z(:,end), z(:,1:3)/glv.min); xygo('\phi_{obs} / ( \prime )');
        subplot(313), plot(phi(:,end), phi(:,1:3)/glv.min); xygo('\phi_{est} / ( \prime )');
        title(sprintf('mu = %.2f; %.2f; %.2f (arcmin)', mu0(1)/glv.min, mu0(2)/glv.min, mu0(3)/glv.min));
    end 
    