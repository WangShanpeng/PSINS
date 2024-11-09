function [mag1, K, b, K1, b1, K0, b0, err] = magellipfit(mag0, isfig)
% Magnetic ellipsoid/circle fitting by nonlinear LS. Magnetic calibration.
%
% Prototype:  [mag1, K, b, K1, b1, K0, b0, err] = magellipfit(mag0, isfig)
% Inputs: mag0 - =[magX, magY, magZ], 3-column magnetic data
%         isfig - result figure flag
% Outputs: mag1 - normalized calibration results, such that
%                mag1 = K1*diag(K0)*(mag0-b0)-b1;
%                     = K*(mag0-b);
%                where K=K1*diag(K0), b=b0+(K1*diag(K0))^-1*b1
%          i.e.  mag1 = delbias(delbias(mag0,b0)*diag(K0)'*K1',b1);
%                     = delbias(mag0,b)*K';
%          K, b, K1, b1, K0, b0, calibration parameters (scale matrix & bias);
%          err - calibration error
%
% See also  magplot, findpeak, lsclbt, lsqcurvefit.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 13/05/2024 
    % [x, idx, idxs] = findpeak([mag0,normv(mag0)], 10, 10);  mag0(idx,:)=[];
    mmax = max(mag0); mmin = min(mag0);
    K0 = 2./(mmax-mmin)';  b0 = (mmax+mmin)'/2;
    mag = delbias(mag0,b0)*diag(K0)';    % coarse calibrate
    %%
    K1 = eye(3);  b1 = zeros(3,1);
    for iter=1:5
        m = delbias(mag*K1', b1);
%         hm = [m(:,1).^2  m(:,1).*m(:,2)  m(:,1).*m(:,3)  m(:,2).^2  m(:,2).*m(:,3)  m(:,3).^2  m];
        hm = [vv2(m), m];
        ym = (sum(m.^2,2)-1)/2;
        [xm, stdx] = lscov(hm, ym);
        K1 = K1 - [xm(1:3)'; 0,xm(4:5)'; 0,0,xm(6)]';
        b1 = b1 + xm(7:9);
    end
    K = K1*diag(K0); b = b0+(K1*diag(K0))^-1*b1;
    mag1 = delbias(mag*K1', b1);
    % mag2 = delbias(mag0,b)*K'; mag3 = delbias(delbias(mag0,b0)*diag(K0)'*K1',b1); myfig; plot([mag2-mag1,mag3-mag1]);
    err = ym - hm*xm;
    if nargin<2, isfig=1; end
    if isfig==1
        myfig
        subplot(321), plot(mag0), xygo('k', 'mag raw');
        subplot(323), plot(mag1), xygo('k', 'mag scaled & fitted'); plot(mag,':');
        subplot(325), plot(err); xygo('k', 'norm fitted error');
        subplot(3,2,[2,4,6])
        r=0.95; [x, y, z] = ellipsoid(0,0,0,r,r,r,20); 
        mesh(x, y, z); hold on; grid on;
        plot3(mag(:,1),mag(:,2),mag(:,3),'b*', mag1(:,1),mag1(:,2),mag1(:,3),'r*');
        xlabel('X'), ylabel('Y'), zlabel('Z');
        axis equal;
    end
    
function v1 = vv2(v)
    [m,n] = size(v); kk=1;
    v1 = zeros(m,n*(n+1)/2);
    for k=1:n
        for k1=k:n
            v1(:,kk) = v(:,k).*v(:,k1); kk=kk+1;
        end
    end