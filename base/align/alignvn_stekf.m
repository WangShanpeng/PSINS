function [att0, attk, xkpk] = alignvn_stekf(imu, qnb, pos, phi0, imuerr, wvn, isfig)
% SINS initial align uses STEKF(State Transformation EKF) with vn as measurement.
% Kalman filter states: 
%    [phiE,phiN,phiU, dvE,dvN,dvU, ebx,eby,ebz, dbx,dby,dbz]'.
%
% Prototype: [att0, attk, xkpk] = alignvn_stekf(imu, qnb, pos, phi0, imuerr, wvn)
% Inputs: imu - IMU data
%         qnb - coarse attitude quaternion
%         pos - position
%         phi0 - initial misalignment angles estimation
%         imuerr - IMU error setting
%         wvn - velocity measurement noise (3x1 vector)
%         isfig - figure flag
% Output: att0 - attitude align result
%         attk, xkpk - for debug
%
% The STEKF source code, lines 45~53, is provided by Maosong Wang.
% Ref. 'State transformation extended Kalman filter for GPS/SINS tightly
%       coupled integration. https://doi.org/10.1007/s10291-018-0773-3'
%
% See also  alignvn, alignvn_ekf.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/10/2021
global glv
    if nargin<4,  phi0 = [1.5; 1.5; 3]*glv.deg;  end
    if nargin<5,  imuerrset(0.01, 100, 0.001, 1);  end
    if nargin<6,  wvn = 0.01;  end;  if length(wvn)==1, wvn=repmat(wvn,3,1); end
    if nargin<7,  isfig = 1; end
    if length(qnb)==3, qnb=a2qua(qnb); end  %if input qnb is Eular angles.
    [nn, ts, nts] = nnts(2, diff(imu(1:2,end)));
    len = fix(length(imu)/nn)*nn;
    eth = earth(pos); vn = zeros(3,1); Cnn = rv2m(-eth.wnie*nts/2);
    kf = avnkfinit(nts, pos, phi0, imuerr, wvn);
    [attk, xkpk] = prealloc(fix(len/nn), 7, 2*kf.n+1);
    ki = timebar(nn, len, 'Initial align using vn as meas (STEKF).');
    for k=1:nn:len-nn+1
        wvm = imu(k:k+nn-1,1:6);  t = imu(k+nn-1,end);
        [phim, dvbm] = cnscl(wvm);
        Cnb = q2mat(qnb);
        dvn = Cnn*Cnb*dvbm;
        vn = vn + dvn + eth.gn*nts;
        qnb = qupdt2(qnb, phim, eth.wnin*nts);
        Fev = [0 -1.0/eth.RMh 0;  1.0/eth.RNh 0 0;  tan(eth.pos(1))/eth.RNh 0 0];  % Mav
        kf.Ft(1:3,1:9) = [-askew(eth.wnin)+Fev*askew(vn), Fev, -Cnb];  
        kf.Ft(4:6,1:12) = [-askew(eth.gn)-askew(vn)*askew(eth.wnie), -askew(2*eth.wnie+eth.wnen), askew(vn)*Cnb, Cnb]; 
        kf.Phikk_1 = kf.I + kf.Ft * nts;
        kf.Hk(:,1:3) = askew(vn);
        kf = kfupdate(kf, vn);
        qnb = qdelphi(qnb, kf.xk(1:3)); kf.xk(1:3) = 0*kf.xk(1:3);
        deta_v = kf.xk(4:6)+askew(vn)*kf.xk(1:3);
        vn = vn-deta_v;  kf.xk(4:6) = 0*kf.xk(4:6);          %^^^
        attk(ki,:) = [q2att(qnb)', vn', t];
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t];
        ki = timebar;
    end
    attk(ki:end,:) = []; xkpk(ki:end,:) = [];
    att0 = attk(end,1:3)';
    resdisp('Initial align attitudes (arcdeg)', att0/glv.deg);
    if isfig, avnplot(attk, xkpk); end
    
function kf = avnkfinit(nts, pos, phi0, imuerr, wvn)
    eth = earth(pos); wnin = eth.wnin;
    kf = []; kf.s = 1; kf.nts = nts;
	kf.Qk = diag([imuerr.web; imuerr.wdb; zeros(6,1)])^2*nts;
    kf.Gammak = 1;
	kf.Rk = diag(wvn)^2/nts;
	kf.Pxk = diag([phi0; [1;1;1]; imuerr.eb; imuerr.db])^2;
	kf.Ft = zeros(12);
	kf.Hk = [zeros(3),eye(3),zeros(3,6)];
    [kf.m, kf.n] = size(kf.Hk);
    kf.I = eye(kf.n);
    kf.xk = zeros(kf.n, 1);
    kf.adaptive = 0;
    kf.xconstrain = 0; kf.pconstrain = 0;
    kf.measmask = [];
    kf.measstop = zeros(kf.m,1);
    kf.measlost = zeros(kf.m,1);
    kf.measlog = 0;
    kf.fading = 1;

function avnplot(attk, xkpk)
global glv
    t = attk(:,end);
    myfigure;
	subplot(421); plot(t, attk(:,1:2)/glv.deg); xygo('pr')
	subplot(423); plot(t, attk(:,3)/glv.deg); xygo('y');
	subplot(425); plot(t, xkpk(:,7:9)/glv.dph); xygo('eb'); 
	subplot(427); plot(t, xkpk(:,10:12)/glv.ug); xygo('db'); 
	subplot(422); plot(t, sqrt(xkpk(:,13:15))/glv.min); xygo('phi');
	subplot(424); plot(t, sqrt(xkpk(:,16:18))); xygo('dV');
	subplot(426); plot(t, sqrt(xkpk(:,19:21))/glv.dph); xygo('eb');
 	subplot(428); plot(t, sqrt(xkpk(:,22:24))/glv.ug); xygo('db');   
