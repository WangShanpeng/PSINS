function [avp, xkpk, zkrk, ins, kf] = sinsgps34(imu, gps, ins, davp, imuerr, lever, dT, rk, Pmin, Rmin, fbstr)
% 19-state SINS/GPS integrated navigation Kalman filter.
% The 19-state includes:
%       phi(3), dvn(3), dpos(3), eb(3), db(3), lever(3), dT(1)
%
% Example1:
% [avp1, xkpk, zkrk, ins1, kf] = sinsgps(imu, gps, 300);
%
% Example2:
% ins = insinit([yaw;pos], ts);
% davp = avperrset([60;300], 1, 100);
% imuerr = imuerrset(0.03, 100, 0.001, 1);
% Pmin = [avperrset([0.1,1],0.001,0.01); gabias(0.1, [10,30]); [0.01;0.01;0.01]; 0.0001].^2;
% Rmin = vperrset(0.001, 0.01).^2;
% [avp1, xkpk, zkrk, ins1, kf] = sinsgps(imu, gps, ins, davp, imuerr, rep3(1), 0.01, vperrset(0.1,10), Pmin, Rmin, 'avp');
%
% See also  sinsgps, kfinit, kfupdate.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/10/2013, 06/02/2021
    [nn, ts, nts] = nnts(2, diff(imu(1:2,end)));
    if size(gps,2)<=5, gpspos_only=1; pos0=gps(1,1:3)'; else, gpspos_only=0; pos0=gps(1,4:6)'; end 
    if ~exist('rk', 'var'),
        if gpspos_only==1, rk=poserrset(10,30);
        else, rk=vperrset([0.1;0.3],[10,30]); end
    end
    if ~exist('dT', 'var'), dT = 0.01; end
    if ~exist('lever', 'var'), lever = rep3(1); end
    if ~exist('imuerr', 'var'), imuerr = imuerrset(0.01, 100, 0.001, 1); end
    if ~exist('davp', 'var'), davp = avperrset([10;300], 1, [10;30]); end
    if ~exist('ins', 'var'), ins=100; end
    if ~isstruct(ins) % T=ins align time
        [~, res0] = aligni0(imu(1:fix(ins/ts),:), pos0);
        ins = insinit([res0.attk(1,1:3)'; 0;0;0; pos0], ts); ins.nts=nts;
    end
    psinstypedef(346-gpspos_only*3);
    kf = [];
    kf.Qt = diag([imuerr.web; imuerr.wdb; zeros(3,1); imuerr.sqg; imuerr.sqa; zeros(3,1); 0; zeros(15,1)])^2;
    kf.Rk = diag(rk)^2;
    kf.Pxk = diag([davp; imuerr.eb; imuerr.db; rep3(lever); dT; imuerr.dKg(1:9)'; imuerr.dKa([1:3,5:6,9])']*1.0)^2;
    kf.Hk = zeros(length(rk),34);
    kf = kfinit0(kf, nts);
    if exist('Pmin', 'var'),
        if length(Pmin)==1, kf.pconstrain=0;
        else, kf.Pmin = Pmin; kf.pconstrain = 1; end
    end
    kf.adaptive = 1;
    if exist('Rmin', 'var'), 
        if length(Rmin)==1, kf.adaptive=0; end
        if kf.adaptive==1, kf.Rmin = diag(Rmin); end
    end
    if exist('fbstr', 'var'), kf.fbstr=fbstr; end
    kf.xtau = [ [1;1;1]; [1;1;1]; [1;1;1]; [1;1;1]; [1;1;1]; [1;1;1]; inf; ones(15,1)];
    imugpssyn(imu(:,7), gps(:,end));
    len = length(imu); [avp, xkpk, zkrk] = prealloc(fix(len/nn), 16, 2*kf.n+1, 2*kf.m+1);
    timebar(nn, len, '34-state SINS/GPS simulation.'); ki = 1; kiz = 1;
    for k=1:nn:len-nn+1
        k1 = k+nn-1; 
        wvm = imu(k:k1,1:6); t = imu(k1,end);
        ins = insupdate(ins, wvm);
        kf.Phikk_1 = kffk(ins);
        kf = kfupdate(kf);
        [kgps, dt] = imugpssyn(k, k1, 'F');
        if kgps>0
            ins = inslever(ins);
            if gpspos_only==1
                zk = ins.posL-gps(kgps,1:3)';
                kf.Hk = [zeros(3,6), eye(3), zeros(3,6), -ins.MpvCnb,-ins.Mpvvn, zeros(3,15)];
            else
                zk = [ins.vnL;ins.posL]-gps(kgps,1:6)';
                kf.Hk = [zeros(6,3), eye(6), zeros(6,6), [-ins.CW,-ins.an;-ins.MpvCnb,-ins.Mpvvn], zeros(6,15)];
            end
            kf = kfupdate(kf, zk, 'M');
            zkrk(kiz,:) = [zk; diag(kf.Rk); t];  kiz = kiz+1;
        end
        [kf, ins] = kffeedback(kf, ins, nts);
        avp(ki,:) = [ins.avp; ins.eb; ins.db; t]';
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]'; ki = ki+1;
        timebar;
    end
    avp(ki:end,:) = []; xkpk(ki:end,:) = [];  zkrk(kiz:end,:) = [];
    insplot(avp);
    kfplot(xkpk);
    rvpplot(zkrk);

