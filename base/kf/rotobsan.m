function [xk, spk, av] = rotobsan(imu, ap0, spk0)
% ROTation OBServability ANalysis for SIMU static rotating operation.
%
% Prototype: [xk, spk, av] = rotobsan(imu, ap0, spk0)
% Inputs: imu - SIMU data after coarse calibration
%         ap0 - initial attitude & position
%         spk0  - initial diag of KF sqrt(P0)
% Outputs: xk - state estimation
%          spk - diag of KF sqrt(Pk)
%          av - INS attitude & velocity
%
% See also  sysclbt, alignpe.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 13/08/2024
global glv
    [nn,ts,nts] = nnts(2, diff(imu(1:2,end))); 
    [wnie, ~, gn] = wnieg(ap0(end-2:end));
    len = length(imu);
    qnb = a2qua(ap0(1:3)); vn = zeros(3,1);
    kf = obskfinit(nts);
    if nargin==3
        if length(spk0)<34    % TIPS: spk0 is the index to remain Pxk no zero, execpt for state 1:6
            spk0 = setdiff(7:34,spk0);
            Pxk = diag(kf.Pxk); Pxk(spk0)=0;
            kf.Pxk = diag(Pxk);
        else
            kf.Pxk = diag(spk0)^2;
        end
    end
    nl=fix(len/nn);  av = zeros(nl,7); xkpk = zeros(nl, kf.n*2+1);  kk = 1;  zk=zeros(nl,4);
    timebar(nn, len, 'IMU rotation observability analysis');
    for k=1:nn:len-20
        k1 = k+nn-1;
        [phim, dvbm] = cnscl(imu(k:k1,1:6)); t = imu(k1,end);
        wb = phim/nts; fb = dvbm/nts;
        fn = qmulv(qnb, fb);
        an = rotv(-wnie*nts/2, fn) + gn;
        vn = vn + an*nts;
        qnb = qupdt2(qnb, phim, wnie*nts);   % insupdate
        kf.Phikk_1 = eye(kf.n)+getFt(fb, wb, q2mat(qnb), wnie)*nts;
        kf = kfupdate(kf);
        if k>20 && mod(kk,10)==0
            ww = mean(imu(k-10:k+10,1:3),1); ww = norm(ww)/ts;
            if ww<100*glv.dph   % if IMU is static
                kf = kfupdate(kf, vn);
                zk(kk,:) = [vn; t]';
            end
        end
        av(kk,:) = [q2att(qnb); vn; t]';
        xkpk(kk,:) = [kf.xk; diag(kf.Pxk); t]'; kk = kk+1;
        timebar;
    end
    av(kk:end,:)=[]; xkpk(kk:end,:)=[];  zk = no0(zk, 1:3);
    xk=xkpk(:,[1:kf.n,end]);  spk=[sqrt(xkpk(:,kf.n+1:end-1)), xkpk(:,end)];
	myfig;    t = xk(:,end);
    subplot(331), plot(av(:,end), av(:,1:3)/glv.deg); xygo('att');
    subplot(332), plot(t, xk(:,1:3)/glv.min); xygo('phi');
    subplot(333), plot(t, xk(:,4:6)); xygo('dV'); plot(av(:,end), av(:,4:6), '--m');
    subplot(334), plot(t, xk(:,7:9)/glv.dph); xygo('eb');
    subplot(335), plot(t, xk(:,10:12)/glv.ug); xygo('db');
    subplot(336), plot(t, xk(:,13:4:21)/glv.ppm); xygo('dKii'); plot(t, xk(:,22:4:30)/glv.ppm, '--');
    subplot(337), plot(t, xk(:,[14:16,18:20])/glv.sec); xygo('dKij'); plot(t, xk(:,[23:25,27:29])/glv.sec, '--');
    subplot(338), plot(t, xk(:,31:33)/glv.ugpg2); xygo('Ka2');
    subplot(339), plot(t, xk(:,34)); xygo('\tau_{GA} / s');
	myfig;    t = spk(:,end);
    subplot(331), plot(zk(:,end), zk(:,1:3), 'x'); xygo('Zk_{vn} / (m/s)');
    subplot(332), plot(t, spk(:,1:3)/glv.min); xygo('phi');
    subplot(333), plot(t, spk(:,4:6)); xygo('dV');
    subplot(334), plot(t, spk(:,7:9)/glv.dph); xygo('eb');
    subplot(335), plot(t, spk(:,10:12)/glv.ug); xygo('db');
    subplot(336), plot(t, spk(:,13:4:21)/glv.ppm); xygo('dKii'); plot(t, spk(:,22:4:30)/glv.ppm, '--');
    subplot(337), plot(t, spk(:,[14:16,18:20])/glv.sec); xygo('dKij'); plot(t, spk(:,[23:25,27:29])/glv.sec, '--');
    subplot(338), plot(t, spk(:,31:33)/glv.ugpg2); xygo('Ka2');
    subplot(339), plot(t, spk(:,34)); xygo('\tau_{GA} / s');

    
function kf = obskfinit(ts)
global glv
    kf.Qt = diag([ [1;1;1]*0.001*glv.dpsh; [1;1;1]*1*glv.ugpsHz; zeros(28,1) ])^2;
    kf.Rk = diag([1;1;1]*0.001)^2;
    kf.Pxk = diag([ [0.1;0.1;1]*glv.deg; [1;1;1]*0.01; [1;1;1]*0.1*glv.dph; [100;100;100]*glv.ug; ...
        [50*glv.ppm;10*glv.sec;10*glv.sec]; [10*glv.sec;60*glv.ppm;10*glv.sec]; [10*glv.sec;10*glv.sec;700*glv.ppm]; ...
        [80*glv.ppm;10*glv.sec;10*glv.sec]; [  0*glv.sec;90*glv.ppm;10*glv.sec]; [  0*glv.sec;  0*glv.sec;99*glv.ppm]; [1;1;1]*100*glv.ugpg2; ...
        0.01 ])^2;
    kf.Hk = [zeros(3),eye(3),zeros(3,28)];
    kf = kfinit0(kf, ts);
    
function Ft = getFt(fb, wb, Cnb, wnie)   % kffk
    o33 = zeros(3); o31 = zeros(3,1);  %wb=[1;2;3]; fb=[1;2;10];
    wX = askew(wnie); fX = askew(Cnb*fb);
    wx = wb(1); wy = wb(2); wz = wb(3); fx = fb(1); fy = fb(2); fz = fb(3);
    CDf2 = Cnb*diag(fb.^2); CwXf = Cnb*cross(wb,fb);
    %        1   4     7    10    13       16       19       22       25       28       31    34
    %states: fi  dvn   eb   db    dKg(:,1) dKg(:,2) dKg(:,3) dKa(:,1) dKa(:,2) dKa(:,3) dKa2  tGA
    Ft = [  -wX  o33  -Cnb  o33  -wx*Cnb  -wy*Cnb  -wz*Cnb   o33      o33      o33      o33   o31
             fX  o33   o33  Cnb   o33      o33      o33      fx*Cnb   fy*Cnb   fz*Cnb   CDf2  CwXf
             zeros(28,34) ];

