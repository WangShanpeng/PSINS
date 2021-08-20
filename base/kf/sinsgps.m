function [avp, xkpk, zkrk, sk, ins, kf] = sinsgps(imu, gps, ins, davp, imuerr, lever, dT, rk, Pmin, Rmin, fbstr)
% 19-state SINS/GPS integrated navigation Kalman filter.
% The 19-state includes:
%       phi(3), dvn(3), dpos(3), eb(3), db(3), lever(3), dT(1)
%
% Example 1:
%   [avp1, xkpk, zkrk, sk, ins1, kf1] = sinsgps(imu, gps, 300);
%
% Example 2:
% ins = insinit([yaw;pos], ts);
% avperr = avperrset([60;300], 1, 100);
% imuerr = imuerrset(0.03, 100, 0.001, 1);
% Pmin = [avperrset([0.1,1],0.001,0.01); gabias(0.1, [10,30]); [0.01;0.01;0.01]; 0.0001].^2;
% Rmin = vperrset(0.001, 0.01).^2;
% [avp1, xkpk, zkrk, sk, ins1, kf1] = sinsgps(imu, gps, ins, avperr, imuerr, rep3(1), 0.01, vperrset(0.1,10), Pmin, Rmin, 'avp');
%
% Example 3:
% t0 = 1;  t1 = 916;
% avp0 = getat(avp,t0);
% ins = insinit(avp0, ts);
% avperr = avperrset([60;300], 1, 10);
% imuerr = imuerrset(0.5, 1000, 0.1, 25);
% Pmin = [avperrset([0.2,1.0],0.01,0.2); gabias(0.01, [10,10]); [0.01;0.01;0.01]; 0.001].^2;
% Rmin = vperrset(0.1, 0.3).^2;
% [avp1, xkpk, zkrk, sk, ins1, kf] = sinsgps(imu(t0/ts:t1/ts,:), gps, ins, avperr, imuerr, rep3(1), 0.1, vperrset(0.1,10), Pmin, Rmin, 'avped');
% 
% See also  kfinit, kfupdate, imugpssyn, igsplot.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/10/2013, 06/02/2021
    [nn, ts, nts] = nnts(2, diff(imu(1:2,end)));
    if size(gps,2)<=5, gpspos_only=1; pos0=gps(1,1:3)'; else, gpspos_only=0; pos0=gps(1,4:6)'; end 
    if ~exist('rk', 'var'),
        if gpspos_only==1, rk=poserrset([10,30]);
        else, rk=vperrset([0.1;0.3],[10,30]); end
    end
    if ~exist('dT', 'var'), dT = 0.01; end
    if ~exist('lever', 'var'), lever = rep3(1); end
    if ~exist('imuerr', 'var'), imuerr = imuerrset(0.01, 100, 0.001, [10;10;100]); end
    if ~exist('davp', 'var'), davp = avperrset([10;300], 1, [10;30]); end
    if ~exist('ins', 'var'), ins=100; end
    if ~isstruct(ins)  % sinsgps(imu, gps, T);  T=ins align time
        [~, res0] = aligni0(imu(1:fix(ins/ts),:), pos0);  imu(1:fix(ins/ts),:)=[];
        ins = insinit([res0.attk(1,1:3)'; 0;0;0; pos0], ts); ins.nts=nts;
    end
    psinstypedef(196-gpspos_only*3);
    kf = [];
    kf.Qt = diag([imuerr.web; imuerr.wdb; zeros(3,1); imuerr.sqg; imuerr.sqa; zeros(3,1); 0])^2;
    kf.Rk = diag(rk)^2;
    kf.Pxk = diag([davp; imuerr.eb; imuerr.db; rep3(lever); dT]*1.0)^2;
    kf.Hk = zeros(length(rk),19);
    kf = kfinit0(kf, nts);
    if exist('Pmin', 'var'),
        if sum(Pmin)<=0, kf.pconstrain=0;
        else kf.Pmin = Pmin; kf.pconstrain = 1; end
    end
    kf.adaptive = 1;
    if exist('Rmin', 'var'), 
        if sum(Rmin)<=0, kf.adaptive=0; end
        if kf.adaptive==1,
            if length(Rmin)==1, kf.Rmin = kf.Rk*Rmin;
            else kf.Rmin = diag(Rmin); end
        end
    end
    if exist('fbstr', 'var'), kf.fbstr=fbstr; end
    kf.xtau = [ [1;1;1]; [1;1;1]; [1;1;1]; [1;1;1]; [1;1;1]; [1;1;1]; 1]*0;
    imugpssyn(imu(:,7), gps(:,end));
    len = length(imu); [avp, xkpk, zkrk, sk] = prealloc(fix(len/nn), 16, 2*kf.n+1, 2*kf.m+1, 2);
    if len<101, return; end;  % return kf struct
    timebar(nn, len, '19-state SINS/GPS simulation.'); ki = 1; kiz = 1;
    for k=1:nn:len-nn+1
        k1 = k+nn-1; 
        wvm = imu(k:k1,1:6); t = imu(k1,end);
        ins = insupdate(ins, wvm);
        kf.Phikk_1 = kffk(ins);
        kf = kfupdate(kf);
        [kgps, dt] = imugpssyn(k, k1, 'F');
        measflag = 0;
        if kgps>0
            ins = inslever(ins); dtpos=+vn2dpos(ins.eth,ins.vnL,ins.tDelay);
            if gpspos_only==1
                measflag = 2;
                zk = ins.posL+dtpos-gps(kgps,1:3)';
                kf.Hk = [zeros(3,6), eye(3), zeros(3,6), -ins.MpvCnb,-ins.Mpvvn];
            else
                measflag = 3;
                zk = [ins.vnL+ins.tDelay*ins.an;ins.posL+dtpos]-gps(kgps,1:6)';
                kf.Hk = [zeros(6,3), eye(6), zeros(6,6), [-ins.CW,-ins.an;-ins.MpvCnb,-ins.Mpvvn]];
            end
            kf = kfupdate(kf, zk, 'M');
            zkrk(kiz,:) = [zk; diag(kf.Rk); t];  kiz = kiz+1;
        end
        [kf, ins] = kffeedback(kf, ins, nts);
        avp(ki,:) = [ins.avp; ins.eb; ins.db; t]';
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';
        sk(ki,:) = [measflag, t]; ki = ki+1;
        timebar;
    end
    avp(ki:end,:) = []; xkpk(ki:end,:) = [];  zkrk(kiz:end,:) = []; sk(ki:end,:) = [];
    insplot(avp);
    kfplot(xkpk);
    rvpplot(zkrk);
    stateplot(sk,2);

