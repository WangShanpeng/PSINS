function [att0, attk, xkpk] = alignvn_ekf(imu, qnb, pos, phi0, imuerr, wvn, isfig)
% SINS initial align uses EKF with vn as measurement.
% EKF 5 states: [phiE,phiN,phiU, dvE,dvN]', 2 measurements: [dvE,dvN]'.
%
% Prototype: [att0, attk, xkpk] = alignvn_ekf(imu, qnb, pos, phi0, imuerr, wvn)
% Inputs: imu - IMU data
%         qnb - coarse attitude quaternion
%         pos - position
%         phi0 - initial misalignment angles estimation
%         imuerr - IMU error setting
%         wvn - velocity measurement noise
%         isfig - figure flag
% Outputs: att0 - attitude align result
%         attk, xkpk - for debug
%
% See also  alignvn, alignvn_stekf, Jacob5.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/10/2021
global glv
    if nargin<4,  phi0 = [1.0; 1.0; 10]*glv.deg;  end
    if nargin<5,  imuerrset(0.01, 100, 0.001, 1);  end
    if nargin<6,  wvn = 0.01;  end;  if length(wvn)==1, wvn=repmat(wvn,3,1); end
    if nargin<7,  isfig = 1; end
    if length(qnb)==3, qnb=a2qua(qnb); end  %if input qnb is Eular angles.
    [nn, ts, nts] = nnts(2, diff(imu(1:2,end)));
    vn = zeros(3,1);
    eth = earth(pos);  Cnn=rv2m(-eth.wnie*nts/2);
    kf = ekfinit(nts, phi0, imuerr, wvn);  % ekf init
    kf.fx = @Jacob5; kf.px.wnie = eth.wnie; kf.px.fn = -eth.gn; kf.px.ts = nts;
    len = length(imu); [attk, xkpk] = prealloc(fix(len/nn), 4, 2*kf.n+1);
    ki = timebar(nn, len, 'Initial align using vn as meas (EKF).' );
    for k=1:nn:len-nn+1
        k1 = k+nn-1;
        wvm = imu(k:k1,1:6);  t = imu(k1,7);
        [phim, dvbm] = cnscl(wvm);
        Cnb = q2mat(qnb);
        dvn = Cnn*Cnb*dvbm;  vn = vn + dvn + eth.gn*nts;   % velocity updating
        qnb = qupdt(qnb, phim-Cnb'*eth.wnie*nts); % attitude updating
        kf.fn = dvn/nts;
        kf = ekf(kf, vn(1:2));
        qnb = qdelafa(qnb, 0.91*kf.xk(1:3)); kf.xk(1:3) = 0.09*kf.xk(1:3);
        vn(1:2) = vn(1:2)-0.91*kf.xk(4:5);  kf.xk(4:5) = 0.09*kf.xk(4:5);
        attk(ki,:) = [q2att(qnb)', t];
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t];
        ki = timebar;
    end
    att0 = attk(end,1:3)';
    resdisp('Initial align attitudes (arcdeg)', att0/glv.deg);
    if isfig, ekfplot(attk, xkpk); end
    
function kf = ekfinit(nts, phi0, imuerr, wvn)
    kf.Qk = diag([imuerr.web; imuerr.wdb(1:2)])^2*nts;
    kf.Rk = diag(wvn(1:2))^2/nts;
    kf.Pxk = diag([phi0; [1;1]])^2;
    kf.Hk = [zeros(2,3), eye(2)];
    kf = kfinit0(kf, nts);
%     [kf.m, kf.n] = size(kf.Hk);
%     kf.I = eye(kf.n);
%     kf.xk = zeros(kf.n, 1);
%     kf.adaptive = 0;
%     kf.xconstrain = 0; kf.pconstrain = 0;
%     kf.fading = 1;
    
function ekfplot(attk, xkpk)
global glv
    t = attk(:,end);
    myfigure;
	subplot(221); plot(t, attk(:,1:2)/glv.deg); xygo('pr')
	subplot(223); plot(t, attk(:,3)/glv.deg); xygo('y');
	subplot(222); plot(t, sqrt(xkpk(:,6:8))/glv.min); xygo('phi');
	subplot(224); plot(t, sqrt(xkpk(:,9:10))); xygo('dV');
    