function [avp, xkpk] = sinsgpscns(imu, ins, gps, qis, utc0, Cbs)
% SINS/GPS/CNS KF with 18-state include:
%       phi(3), dvn(3), dpos(3), eb(3), db(3), mu(3)
% 9-measurement include:
%       dvn(3), dpos(3), phi(3)
%
% See also  sinsgps, test_SINS_CNS_184.m.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/11/2022
global glv
    if nargin<6, Cbs=eye(3); end
    [nn, ts, nts] = nnts(2, diff(imu(1:2,end)));
    if ~isstruct(ins)
        att = aligni0vn(imu(1:fix(ins/ts),:), gps(1,4:6)');  imu(1:fix(ins/ts),:) = [];
        ins = insinit([att; gps(1,1:6)'], ts);
    end
    Cie0 = cnsCie(utc0(1:3), utc0(4));
    % KF setting
    psinstypedef('test_SINS_CNS_def');
    davp0 = avperrset([0.5;-0.5;20], 0.1, [1;1;3]*10);
    imuerr = imuerrset(0.01, 100, 0.001, 10);
    mu = [10;10;30]*glv.min;
    rk = [30;30;60]*glv.sec;
    kf = kfinit(ins, davp0, imuerr, rk, mu);
    kf.Hk = [zeros(6,3), eye(6), zeros(6,9); eye(3), zeros(3,12), ins.Cnb]; kf.Hk(7,7)=1; kf.Hk(8:9,8) = -[ins.eth.cl; ins.eth.sl]; [kf.m, kf.n] = size(kf.Hk);
    kf = kfinit0(kf, nts);
    kf.Rk = diag([0.1;0.1;0.1; 10;10;30; rk])^2;  kf.Rmin = 0.01*kf.Rk;  kf.Rmax = 100*kf.Rk; kf.adaptive=1;
    kf.Pmin = [avperrset([0.3,1],0.01,0.1); gabias(0.01, [10,30]); [1;1;3]*glv.min].^2;  kf.pconstrain=1;
    kf.measmask = 9;
    % alloc memory
    len = length(imu);
    [avp, xkpk, measlog] = prealloc(fix(len/nn), 10, 2*kf.n+1, 2);
    ki = timebar(nn, len, '18-state SINS/GPS/CNS KF processing.');
    imugpssyn(imu(:,end), gps(:,end));    imugpssyn1(imu(:,end), qis(:,end));
    for k=1:nn:len-nn+1
        k1 = k+nn-1;
        wvm = imu(k:k1,1:6); t = imu(k1,end);
        ins = insupdate(ins, wvm);
        kf.Phikk_1 = kffk(ins);
        kf = kfupdate(kf);
        [kgps, dt] = imugpssyn(k, k1, 'F');
        if kgps>0 && norm(ins.wnb)<10*glv.dps  % SINS/GPS
            zk = [ins.vn-gps(kgps,1:3)'; ins.pos-gps(kgps,4:6)'];
            kf = kfupdate(kf, [zk;vrbig(3,1)], 'M');
        end
        [kcns, dt] = imugpssyn1(k, k1, 'F');
        if kcns>0 && norm(ins.wnb)<5*glv.dps  % SINS/CNS
            kf.Hk(8:9,8) = -[ins.eth.cl; ins.eth.sl];
            Cns = cnsCns(qis(kcns,1:4)', ins.pos, Cie0, qis(kcns,5), Cbs);  % Cnb(star)
            zk = qq2phi(ins.qnb, m2qua(Cns));
            if max(abs(zk))<1*glv.deg
                kf = kfupdate(kf, [vrbig(6,1);zk], 'M');
            end
        end
        [kf, ins] = kffeedback(kf, ins, 1, 'avp');
        % save results
        avp(ki,:)  = [ins.avp', t];
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';
        measlog(ki,:) = [kf.measlog; t]';  kf.measlog=0;
        ki = timebar;
    end
    [avp,xkpk] = no0s(avp,xkpk);
    insplot(avp);
    kfplot(xkpk);
    stateplot(measlog,kf.m);

