function [avp, xkpk, zkrk, sk, ins, kf] = sinsgps(imu, gps, ins, davp, imuerr, lever, dT, rk, Pmin, Rmin, fbstr, isfig)
% 19-state SINS/GNSS integrated navigation Kalman filter.
% The 19-state includes:
%       [phi(3); dvn(3); dpos(3); eb(3); db(3); lever(3); dT(1)]
% The 3- or 6- measurement is:
%       [dpos(3)] or [dvn(3); dpos(3)]
%
% Prototype: [avp, xkpk, zkrk, sk, ins, kf] = sinsgps(imu, gps, ins, davp, imuerr, lever, dT, rk, Pmin, Rmin, fbstr, isfig)
% Inputs: imu - IMU array [wm, vm, t]
%         gps - GNSS array [vn, pos, t] or [pos, t];
%         ins - ins array, set by function 'insinit', or double to indicate an align time
%         davp - AVP array for P0 setting
%         imuerr - set by function 'imuerrset', for P0 and Qk setting
%         lever - lever arm from IMU to GNSS,
%                 if lever(4)=0 then Pk(lever)=0 for no lever estimation, =1 for lever estimation
%                 if lever(5)=0 then AVP output IMU position, else lever(5)=1 for GNSS antenna position
%         dT - time delay from IMU to GNSS, 
%                 if dT(2)=0 then Pk(dT)=0 for no time delay estimation
%         rk - measurement noise std(dpos) or std([dvn;dpos])
%         Pmin - Pmin setting, Pmin<=0 for no Pmin constrain
%         Rmin - Rmin setting, Rmin<=0 for no adaptive KF, Rmin=0~1 scale for adaptive KF and Rmin = Rk*Rmin
%         fbstr - KF feedback string from any combination of 'avpedLT'
%         isfig - figure flag
%
% Example 1:
%   [avp1, xkpk, zkrk, sk, ins1, kf1] = sinsgps(imu, gps, 300);
%
% Example 2:  RLG/FOG
% ins = insinit([yaw;pos], ts);
% avperr = avperrset([60;300], 1, 100);
% imuerr = imuerrset(0.03, 100, 0.001, 1);
% Pmin = [avperrset([0.1,1],0.001,0.01); gabias(0.001, [10,30]); [0.01;0.01;0.01]; 0.0001].^2;
% Rmin = vperrset(0.001, 0.01).^2;
% [avp1, xkpk, zkrk, sk, ins1, kf1] = sinsgps(imu, gps, ins, avperr, imuerr, [rep3(1);1;1], [0.01;1], vperrset(0.1,10), Pmin, Rmin, 'avp');
%
% Example 3:  FOG/MEMS
% t0 = 1;  t1 = 916;
% avp0 = getat(avp,t0);
% ins = insinit(avp0, ts);
% avperr = avperrset([60;300], 1, 10);
% imuerr = imuerrset(0.5, 1000, 0.1, 25);
% Pmin = [avperrset([0.2,1.0],0.01,0.2); gabias(0.01, [10,10]); [0.01;0.01;0.01]; 0.001].^2;
% Rmin = vperrset(0.1, 0.3).^2;
% [avp1, xkpk, zkrk, sk, ins1, kf] = sinsgps(imu(t0/ts:t1/ts,:), gps, ins, avperr, imuerr, rep3(1), 0.1, vperrset(0.1,10), Pmin, Rmin, 'avped');
% 
% See also  kfinit, kfupdate, imugpssyn, igsplot, insupdate, posprocessing.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/10/2013, 06/02/2021, 02/11/2021, 30/08/2022
global glv
    [nn, ts, nts] = nnts(2, diff(imu(1:2,end)));
    clmgps = size(gps,2); SatNum = 20; DOP = 1.0;
    if clmgps<=5, gpspos_only=1; pos0=gps(1,1:3)'; else, gpspos_only=0; pos0=gps(1,4:6)'; end 
    if ~exist('rk', 'var'),
        if gpspos_only==1, rk=poserrset([10,30]);
        else, rk=vperrset([0.1;0.3],[10,30]); end
    end
    if ~exist('dT', 'var'), dT = 0.01; end;   if length(dT)==1, dT(2,1)=1; end
    if ~exist('lever', 'var'), lever = rep3(1); else, lever=rep3(lever); end;   if length(lever)==3, lever(4)=1; end;   if length(lever)<5, lever(5)=1; end
    if ~exist('imuerr', 'var'), imuerr = imuerrset(0.05, 500, 0.001, [10;10;100]); end
    if ~exist('davp', 'var'), davp = avperrset([10;300], 1, [10;30]); end
    if ~exist('ins', 'var'), ins=100; end
    if ~isstruct(ins)  % sinsgps(imu, gps, T);  T=ins align time
        [~, res0] = aligni0(imu(1:fix(ins/ts),:), pos0);  imu(1:fix(ins/ts),:)=[];
        ins = insinit([res0.attk(1,1:3)'; 0;0;0; pos0], ts); ins.nts=nts;
    end
    ins.lever = lever(1:3)*(1-lever(4));  ins.tDelay = dT(1)*(1-dT(2));
    ins = inslever(ins, -ins.lever);  ins.vn = ins.vnL; ins.pos = ins.posL;
    if ~isempty(glv.dgn), ins.eth = attachdgn(ins.eth, glv.dgn); end
    psinstypedef(196-gpspos_only*3);
    kf = [];
    kf.Qt = diag([imuerr.web; imuerr.wdb; zeros(3,1); imuerr.sqg; imuerr.sqa; zeros(3,1); 0])^2;
    kf.Rk = diag(rk)^2;
    kf.Pxk = diag([davp; imuerr.eb; imuerr.db; lever(1:3)*lever(4); dT(1)*dT(2)]*1.0)^2;   % 2021/11/2
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
    kf.xtau = [ [1;1;1]; [1;1;1]; [1;1;1]; [1;1;1]; [1;1;1]; [1;1;1]; 1]*1;
    imugpssyn(imu(:,end), gps(:,end));
    len = length(imu); [avp, xkpk, zkrk, sk] = prealloc(fix(len/nn), 16, 2*kf.n+1, 2*kf.m+1, 2);
    if len<101, return; end;  % return kf struct
    timebar(nn, len, '19-state SINS/GNSS simulation.'); ki = 1; kiz = 1;
    for k=1:nn:len-nn+1
        k1 = k+nn-1; 
        wvm = imu(k:k1,1:6); t = imu(k1,end);
        ins = insupdate(ins, wvm);  ins.eth.dgnt=t;
        kf.Phikk_1 = kffk(ins);
        kf = kfupdate(kf);
        [kgps, dt] = imugpssyn(k, k1, 'F');
        ins = inslever(ins); 
        if kgps>0 && (clmgps==5||clmgps==8)  % having SatNum.DOP
            SatNum = gps(kgps,end-1); DOP = (SatNum-fix(SatNum))*1000;
            if SatNum<10||DOP>1.5, kgps=0; end  % disable meas
        end
        if kgps>0
            dtpos=+vn2dpos(ins.eth,ins.vnL,ins.tDelay);
            if gpspos_only==1
                zk = ins.posL+dtpos-gps(kgps,1:3)';
                kf.Hk = [zeros(3,6), eye(3), zeros(3,6), -ins.MpvCnb,-ins.Mpvvn];
            else
                zk = [ins.vnL+ins.tDelay*ins.anbar;ins.posL+dtpos]-gps(kgps,1:6)';
%                 if gps(kgps,7)<10 || (gps(kgps,7)-fix(gps(kgps,7)))*1000>1.2
%                     zv = ins.Cnb'*zk(1:3); zv(1)=0; zp = ins.MpvCnb^-1*zk(4:6); %zp(1)=0;
%                     zk = [ins.Cnb*zv; ins.MpvCnb*zp];
%                 end
                kf.Hk = [zeros(6,3), eye(6), zeros(6,6), [-ins.CW,-ins.anbar;-ins.MpvCnb,-ins.Mpvvn]];
            end
            kf = kfupdate(kf, zk, 'M');
            zkrk(kiz,:) = [zk; diag(kf.Rk); t];  kiz = kiz+1;
        end
        [kf, ins] = kffeedback(kf, ins, nts);
        if lever(5)==1,  avp(ki,:) = [ins.att; ins.vnL; ins.posL; ins.eb; ins.db; t]';
        else             avp(ki,:) = [ins.att; ins.vn;  ins.pos;  ins.eb; ins.db; t]';        end
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';
        sk(ki,:) = [kf.measlog, t];  kf.measlog=0;  ki = ki+1; 
        timebar;
    end
    avp(ki:end,:) = []; xkpk(ki:end,:) = [];  zkrk(kiz:end,:) = []; sk(ki:end,:) = [];
    if ~exist('isfig', 'var'), isfig=1; end
    if isfig==1
        insplot(avp);
        kfplot(xkpk);
        rvpplot(zkrk);
        stateplot(sk,length(zk)/3);
    end
