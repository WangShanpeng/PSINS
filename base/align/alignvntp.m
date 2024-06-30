function [att0, attk, xkpk] = alignvntp(imu, qnb, pos, phi0, imuerr, wvn, isfig)
% Two position initial align uses Kalman filter with vn as measurement.
% Ref. alignvn.
%    imu = [wm, vm, dp_flag, t]
%
% See also  alignsbtp, angaddplot, alignvn, vn2phi.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 20/10/2022
global glv
    if nargin<4,  phi0 = [1.5; 1.5; 3]*glv.deg;  end
    if nargin<5,  imuerr = imuerrset(0.01, 100, 0.001, 1);  end
    if nargin<6,  wvn = 0.01;  end;  if length(wvn)==1, wvn=repmat(wvn,3,1); end
    if nargin<7,  isfig = 1; end
    if length(qnb)==3, qnb=a2qua(qnb); end  %if input qnb is Eular angles.
    [nn, ts, nts] = nnts(2, diff(imu(1:2,end)));
    len = fix(length(imu)/nn)*nn;
    eth = earth(pos); vn = zeros(3,1); Cnn = rv2m(-eth.wnie*nts/2);
    kf = avnkfinit(nts, pos, phi0, imuerr, wvn);
    [attk, xkpk] = prealloc(fix(len/nn), 7, 2*kf.n+1);
    ki = timebar(nn, len, 'Initial align using vn as meas.');
    if size(imu,2)<8,
        w = normv(imu(:,1:3))/ts;
        idx = find(w>1*glv.dps);
        dpflag = w*0; dpflag(1:idx(1)-100)=1; dpflag(idx(end)+100:end)=2;
    end;
    tp = 0; P0 = kf.Pxk;
    for k=1:nn:len-nn+1
        wvm = imu(k:k+nn-1,1:6);  t = imu(k+nn-1,end);
        [phim, dvbm] = cnscl(wvm);
        Cnb = q2mat(qnb);
        dvn = Cnn*Cnb*dvbm;
        vn = vn + dvn + eth.gn*nts;
        qnb = qupdt2(qnb, phim, eth.wnin*nts);
        Cnbts = Cnb*nts;
        kf.Phikk_1(4:6,1:3) = askew(dvn);
            kf.Phikk_1(1:3,7:9) = -Cnbts; kf.Phikk_1(4:6,10:12) = Cnbts;
        kf = kfupdate(kf);
        if dpflag(k)>0
%             if tp==0&&dpflag(k)==2, vn = zeros(3,1); kf.xk(4:6)=0; tp = 1; end
%             if tp==0&&dpflag(k)==2, vn = zeros(3,1); tp = 1; end
            if tp==0&&dpflag(k)==2, for ii=[1:2,4:6,10:12], kf.Pxk(ii,ii)=P0(ii,ii); end; tp = 1; end
            kf = kfupdate(kf, vn, 'M');
        end
        if dpflag(k)>0
            qnb = qdelphi(qnb, 0.91*kf.xk(1:3)); kf.xk(1:3) = 0.09*kf.xk(1:3);
            vn = vn-0.91*kf.xk(4:6);  kf.xk(4:6) = 0.09*kf.xk(4:6);
        end
        attk(ki,:) = [q2att(qnb); vn; t]';
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t];
        ki = timebar;
    end
    attk(ki:end,:) = []; xkpk(ki:end,:) = [];
    att0 = attk(end,1:3)';
    resdisp('Initial align attitudes (arcdeg)', att0/glv.deg);
    if isfig, avnplot(attk, xkpk); end
    
function kf = avnkfinit(nts, pos, phi0, imuerr, wvn)
    eth = earth(pos); wnie = eth.wnie;
    kf = []; kf.s = 1; kf.nts = nts;
	kf.Qk = diag([imuerr.web; imuerr.wdb; zeros(6,1)])^2*nts;
    kf.Gammak = 1;
	kf.Rk = diag(wvn)^2/nts;
	kf.Pxk = diag([phi0; [1;1;1]; imuerr.eb; imuerr.db])^2;
	Ft = zeros(12); Ft(1:3,1:3) = askew(-wnie); kf.Phikk_1 = eye(12)+Ft*nts;
	kf.Hk = [zeros(3),eye(3),zeros(3,6)];
    [kf.m, kf.n] = size(kf.Hk);
    kf.I = eye(kf.n);
    kf.xk = zeros(kf.n, 1);
    kf.adaptive = 0;
    kf.xconstrain = 0; kf.pconstrain = 0;
    kf.fading = 1;

function avnplot(attk, xkpk)
global glv
    t = attk(:,end);
    myfigure;
	subplot(421); plot(t, attk(:,1:2)/glv.deg); xygo('pr'); title('Xi');
	subplot(423); plot(t, attk(:,3)/glv.deg); xygo('y');
	subplot(425); plot(t, xkpk(:,7:9)/glv.dph); xygo('eb'); 
	subplot(427); plot(t, xkpk(:,10:12)/glv.ug); xygo('db'); 
	subplot(422); plot(t, sqrt(xkpk(:,13:15))/glv.min); xygo('phi'); title('\surdPii')
	subplot(424); plot(t, sqrt(xkpk(:,16:18))); xygo('dV');
	subplot(426); plot(t, sqrt(xkpk(:,19:21))/glv.dph); xygo('eb');
 	subplot(428); plot(t, sqrt(xkpk(:,22:24))/glv.ug); xygo('db');   
