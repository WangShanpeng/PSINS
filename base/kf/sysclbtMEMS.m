function [clbt, av, avi] = sysclbtMEMS(imu, pos0, yaw0, itertion, isKap, wStatic)
% SIMU systemtic calibration processing under specific rotating operation.
%
% Prototype: [clbt, av] = sysclbtMEMS(imu, pos0, yaw0, itertion)
% Inputs: imu - SIMU data after coarse calibration
%         pos0 - geographical position = [latitude; longitude; height]
%         yaw0 - initial yaw angle.
%         itertion - calibration processing itertion times.
%         isKap - Kap parameter calibration flag.
%         wStatic - static angular threshold.
% Output: clbt - SIMU fine calibration result array after this porcessing,
%               inculde fields:
%                 'Kg, Ka, eb, db, Kap(3x1), gSens(3x3), such that
%                 wm = Kg*wm - (eb + gSens*fb)*ts
%                 vm = Ka*vm - (db + Kap.*abs(fb))*ts
%                where wb=wm/ts, fb=vm/ts. 
%
% See also  sysclbt, imuscale, imudpdrift, lsclbt, clbtfile, clbtdiff, imuclbt, imuerrset, kfupdate.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/04/2024
global glv
    if nargin<6, wStatic=0.1*glv.dps; end
    if nargin<5, isKap=0; end
    if nargin<4, itertion=5; end
    if nargin<3, yaw0=0; end
    if length(pos0)<3, pos0=[pos0(1);0;0]; end
    [wnie, g0, gn] = wnieg(pos0);
    [nn,ts,nts] = nnts(2, diff(imu(1:2,end))); 
    tk1 = fix(1/ts); tk2 = fix(10/ts);
    att = alignsb(imu(tk1:tk2,:), pos0);
    clbt.eb = mean(imu(tk1:tk2,1:3))'/ts-a2mat(att)'*wnie;
    clbt.Kg = eye(3); clbt.Ka = eye(3); clbt.Kap = zeros(3,1); clbt.db = zeros(3,1);  clbt.gSens = zeros(3);
    ww = normv(ar1filt(delbias(imu(:,1:3),clbt.eb*ts),fix(0.2/ts)))/ts; wH = wStatic; wmeas = ww; % myfig, plot(imu(:,end), w/glv.dps);
    len = length(imu);  imu1 = imu;
    for iter=1:itertion
        imu1(:,1:6) = [imu(:,1:3)*clbt.Kg', imu(:,4:6)*clbt.Ka'];    % IMU calibration for alignment
        gS = imu(:,4:6)*clbt.gSens';
        imu1(:,1:6) = [imu1(:,1)-clbt.eb(1)*ts-gS(:,1),imu1(:,2)-clbt.eb(2)*ts-gS(:,2),imu1(:,3)-clbt.eb(3)*ts-gS(:,3), ...  % 20181102
                       imu1(:,4)-clbt.db(1)*ts,imu1(:,5)-clbt.db(2)*ts,imu1(:,6)-clbt.db(3)*ts];
        qnb = a2qua(alignsb(imu1(tk1:tk2,:), pos0)); vn = zeros(3,1);  qnb=setyaw(qnb,yaw0); % align
        dotwf = imudot(imu1, 5.0);
%         if iter~=itertion,  kf = clbtkfinit(nts);  else, kf.Pxk = kf.Pxk*100; kf.Pxk(:,3)=0; kf.Pxk(3,:)=0; kf.xk = kf.xk*0; end
%         if iter~=itertion,  kf = clbtkfinit(nts);  else, kf.Pxk = diag(diag(kf.Pxk))*100; kf.xk = kf.xk*0; end
        kf = clbtkfinit(nts);
        kf.Pxk(31:33,31:33)=kf.Pxk(31:33,31:33)*isKap;
        t1s = 0; vn1s = zeros(fix(len*ts), 5);  kkv = 1;  qnb_1=2;
        av = zeros(fix(len*ts),7); xkpk = zeros(fix(len*ts), kf.n*2+1);  kk = 1;
        timebar(nn, len-2*tk1, sprintf('System Calibration of SIMU( iter=%d ).',iter));
        for k=tk1:nn:len-tk1
            k1 = k+nn-1;
            wm = imu(k:k1,1:3); vm = imu(k:k1,4:6); t = imu(k1,end);
            [phim, dvbm] = cnscl([wm,vm]);
            phim = clbt.Kg*phim-clbt.eb*nts-clbt.gSens*dvbm; dvbm = clbt.Ka*dvbm-clbt.db*nts;
            wb = phim/nts; fb = dvbm/nts;
            fn = qmulv(qnb, fb-clbt.Kap.*abs(fb));
            an = rotv(-wnie*nts/2, fn) + gn;
            vn = vn + an*nts;
            qnb = qupdt2(qnb, phim, wnie*nts);   % insupdate
            t1s = t1s + nts;
            Ft = getFt(fb, wb, q2mat(qnb), wnie);   kf.Phikk_1 = eye(kf.n)+Ft*nts;
            kf = kfupdate(kf);
            if t1s>(0.2-ts/2)  % kf measurement update every 0.2 second
                if max(ww(k-5:k+5))<wH && qnb_1(1)<=1  % if IMU is static
                    wmeas(k) = wH;  phi = qq2phi(qnb,qnb_1);
                    zk = [vn; phi(3)/t1s]; kf.Hk(4,:) = Ft(3,:);   % ZIHR
                    kf = kfupdate(kf, zk);
                    vn1s(kkv,:) = [zk; t]';  kkv = kkv+1;
                end
                t1s = 0;  qnb_1 = qnb;
                av(kk,:) = [q2att(qnb); vn; t]';
                xkpk(kk,:) = [kf.xk; diag(kf.Pxk); t]'; kk = kk+1;
            end
            timebar;
        end
        if iter~=itertion,  clbt = clbtkffeedback(kf, clbt);  end
        vn1s(kkv:end,:) = []; av(kk:end,:) = []; xkpk(kk:end,:) = [];
        clbtkfplot(av, xkpk, vn1s, imu, dotwf, iter);
        if iter==1,  myfig, plot(imu(:,end), [ww,wmeas]/glv.dps); xygo('w');  end
        if nargout==3, avi{iter} = av; end
    end
	subplot(339), plot(vn1s(:,end),vn1s(:,4)/glv.dph); xygo('\phi_U^{dot} / \circ/h');
    clbtkfplot(av, xkpk, vn1s, imu, dotwf, 100);
    if isKap==0, clbt=rmfield(clbt,'Kap'); end
    
function kf = clbtkfinit(ts)
global glv
    kf.Qt = diag([ [1;1;1]*0.1*glv.dpsh; [1;1;1]*100*glv.ugpsHz; zeros(36,1) ])^2;
    kf.Rk = diag([[1;1;1]*0.01; 10*glv.dph])^2;
    kf.Pxk = diag([ [1;1;1/10]*glv.deg; [1;1;1]; [10;10;10]*1*glv.dph; [1;1;1]*glv.mg; ...
        [100*glv.ppm;100*glv.sec;100*glv.sec]; [100*glv.sec;100*glv.ppm;100*glv.sec]; [100*glv.sec;100*glv.sec;100*glv.ppm]; ...
        [100*glv.ppm;100*glv.sec;100*glv.sec]; [  0*glv.sec;100*glv.ppm;100*glv.sec]; [  0*glv.sec;  0*glv.sec;100*glv.ppm]; [100;100;100]*0*glv.ppm; ...
        [10;10;10]*glv.dphpg; [10;10;10]*glv.dphpg; [10;10;10]*glv.dphpg ])^2;
    kf.Hk = [[zeros(3),eye(3),zeros(3,36)]; [0,0,1,zeros(1,39)]];
    kf = kfinit0(kf, ts);
    
function clbt = clbtkffeedback(kf, clbt)   % kffeedback
    clbt.Kg = (eye(3)-reshape(kf.xk(13:21),3,3))*clbt.Kg;
    clbt.Ka = (eye(3)-reshape(kf.xk(22:30),3,3))*clbt.Ka; clbt.Kap = clbt.Kap+kf.xk(31:33);
    clbt.eb = clbt.eb+kf.xk(7:9); clbt.db = clbt.db+kf.xk(10:12);
    clbt.gSens = clbt.gSens+reshape(kf.xk(34:42),3,3);

function Ft = getFt(fb, wb, Cnb, wnie)   % kffk
    o33 = zeros(3); %wb=[1;2;3]; fb=[1;2;10];
    wX = askew(wnie); fX = askew(Cnb*fb);
    wx = wb(1); wy = wb(2); wz = wb(3); fx = fb(1); fy = fb(2); fz = fb(3);
    CDpf = Cnb*diag(abs(fb));
    %        1   4     7    10    13       16       19       22       25       28       31     34          37           40    
    %states: fi  dvn   eb   db    dKg(:,1) dKg(:,2) dKg(:,3) dKa(:,1) dKa(:,2) dKa(:,3) dKap   gSens(:,1)  gSensy(:,2)  gSensz(:,3)
    Ft = [  -wX  o33  -Cnb  o33  -wx*Cnb  -wy*Cnb  -wz*Cnb   o33      o33      o33      o33   -fx*Cnb     -fy*Cnb      -fz*Cnb
             fX  o33   o33  Cnb   o33      o33      o33      fx*Cnb   fy*Cnb   fz*Cnb   CDpf   o33         o33          o33
             zeros(36,42) ];
    
function clbtkfplot(av, xkpk, vn1s, imu, dotwf, iter)
global glv
    if iter==1
        myfig
        subplot(221), plot(av(:,end), av(:,1:3)/glv.deg); xygo('att');
        subplot(223), plot(av(:,end), av(:,4:6), vn1s(:,end),vn1s(:,1:3),'*'); xygo('V');
        subplot(2,2,2), plot(dotwf(:,end), dotwf(:,1:3)/glv.deg, ':'), xygo('\omega / \circ/s, d\omega/dt / \circ/s^2');
            hold on, plot(imu(:,end), imu(:,1:3)/diff(imu(1:2,end))/glv.dps, vn1s(:,end),vn1s(:,3)*0,'*');
        subplot(224), plot(vn1s(:,end),vn1s(:,4)/glv.dph); xygo('\phi_U^{dot} / \circ/h');
    end
    if iter==100
        plotxk([sqrt(xkpk(:,[43:end-1])),xkpk(:,end)]);
    else
        plotxk(xkpk(:,[1:42,end]));  subplot(332), plot(av(:,end), av(:,4:6), 'm');
    end
 
function plotxk(xk)        
global glv
    myfig
    t = xk(:,end);
    subplot(331), plot(t, xk(:,1:3)/glv.min); xygo('phi');
    subplot(332), plot(t, xk(:,4:6)); xygo('dV')
    subplot(333), plot(t, xk(:,7:9)/glv.dph); xygo('eb');
    subplot(334), plot(t, xk(:,10:12)/glv.ug); xygo('db');
    subplot(335), plot(t, xk(:,13:4:21)/glv.ppm); xygo('dKii');
        hold on,  plot(t, xk(:,22:4:30)/glv.ppm, '--');
    subplot(336), plot(t, xk(:,[14:16,18:20])/glv.sec); xygo('dKij');
    	hold on,  plot(t, xk(:,[23:25,27:29])/glv.sec, '--');
    subplot(337), plot(t, xk(:,31:33)/glv.ppm); xygo('Kap');
    subplot(338), plot(t, xk(:,34:36)/glv.dphpg); xygo('gS');
    	hold on,  plot(t, xk(:,37:39)/glv.dphpg,'--'); plot(t, xk(:,40:42)/glv.dphpg, '-.');

