% SINS/GPS/CNS centralized v.s. federated KF with 15-state include:
%       phi(3), dvn(3), dpos(3), eb(3), db(3)
% Please run 'test_SINS_trj.m' to generate 'trj10ms.mat' beforehand!!!
% See also  sinsgps, test_SINS_GPS_156, test_SINS_CNS_184.
% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/12/2021
glvs
trj = trjfile('trj10ms.mat');
[nn, ts, nts] = nnts(2, trj.ts);
% GPS simulator
lever = [1; 2; 3]*0; rk1 = vperrset(0.1, 1);
davp0 = avperrset([0.5;-0.5;20], 0.01, [1;1;3]);
gps = gpssimu(trj.avp, davp0(4:6), davp0(7:9), 1, lever, 0.0);  % gpsplot(gps)
% CNS simulator
mu = 0*[1;2;5]*glv.min;  rk2 = [[10;10;30]*glv.sec];
[qis, utc0] = cnssimu(trj.avp, rk2, mu, [2021;11;22;12*3600; -0.1;37]);
% SINS init
imuerr = imuerrset(0.03, [100;100;100], 0.001, 5);
imu = imuadderr(trj.imu, imuerr);  % imuplot(imu)
ins = insinit(trj.avp0(1:9), ts, davp0);  len = length(imu);
Cie0 = cnsCie(utc0(1:3), utc0(4),  utc0(5), utc0(6));
% centralized KF
psinstypedef(156);
kf = [];  kf1 = []; kf2 = [];
kf.Qt = diag([imuerr.web; imuerr.wdb; zeros(9,1)])^2;  % 15-state
kf.Rk = diag([rk1; rk2])^2;
kf.Pxk = diag([davp0; imuerr.eb; imuerr.db]*1.0)^2;
kf.Hk = [ zeros(6,3), eye(6), zeros(6,6); ...    % SINS/GPS Hk
          eye(3), zeros(3,12) ];  kf.Hk(7,7)=1;  % SINS/CNS Hk
kf = kfinit0(kf, nts); 
% SINS/GPS KF1
kf1.fbeta = 1/2;
kf1.Qt = kf.Qt;  kf1.Pxk = kf.Pxk;
kf1.Rk = diag(rk1)^2;
kf1.Hk = [zeros(6,3), eye(6), zeros(6,6)];
kf1 = kfinit0(kf1, nts);
% SINS/CNS KF2
kf2.fbeta = 1/3;
kf2.Qt = kf.Qt;   kf2.Pxk = kf.Pxk;
kf2.Rk = diag(rk2)^2;
kf2.Hk = [eye(3), zeros(3,12)];  kf2.Hk(1,7)=1;
kf2 = kfinit0(kf2, nts);
% feterated KF for SINS/GPS & SINS/CNS
fins = ins;
fkf = kf2;  fkf.fbeta = 1-kf1.fbeta-kf2.fbeta;
if fkf.fbeta<0 || fkf.fbeta>1, error('FKF info-factor must be within [0,1] !'); end
% 
[avp,  xkpk]  = prealloc(fix(len/nn), 10, 2*kf.n+1);
[avp1, xkpk1] = prealloc(fix(len/nn), 10, 2*kf1.n+1);
[avp2, xkpk2] = prealloc(fix(len/nn), 10, 2*kf2.n+1);
[avpf, xkpkf] = prealloc(fix(len/nn), 10, 2*fkf.n+1);
ki = timebar(nn, len, '15-state SINS/GPS/CNS centralized v.s. federated KF simulation.');
imugpssyn(imu(:,7), gps(:,end)); 
for k=1:nn:len-nn+1
    k1 = k+nn-1;
    wvm = imu(k:k1,1:6); t = imu(k1,end);
    % SINS/GPS/CNS sequential centralized KF
    ins = insupdate(ins, wvm);
    kf.Phikk_1 = kffk(ins);
    kf = kfupdate(kf);
    [kgps, dt] = imugpssyn(k, k1, 'F');
    if kgps>0
        zk1 = [ins.vn-gps(kgps,1:3)'; ins.pos-gps(kgps,4:6)'];
        kf.Hk = kf1.Hk;  kf.Rk = kf1.Rk;   kf = kfupdate(kf, zk1, 'M');
    end
    if mod(t,1)<1.5*ts && norm(ins.wnb)<1*glv.dps
        kf2.Hk(2:3,8) = -[ins.eth.cl; ins.eth.sl];
        Cns = cnsCns(qis(k1,1:3)', ins.pos, Cie0, t);
        zk2 = qq2phi(ins.qnb,m2qua(Cns));
        kf.Hk = kf2.Hk;  kf.Rk = kf2.Rk;   kf = kfupdate(kf, zk2, 'M');
    end
    [kf, ins] = kffeedback(kf, ins, 1, 'avp');
    % SINS/GPS feterated sub-KF1
    fins = insupdate(fins, wvm);
    kf1.Pxk = fkf.Pxk;  kf1.xk = fkf.xk;  % sub-KF1 reset
    kf1.Phikk_1 = kffk(fins);
    kf1 = kfupdate(kf1);
    if kgps>0
        zk1 = [fins.vn-gps(kgps,1:3)'; fins.pos-gps(kgps,4:6)'];
        kf1 = kfupdate(kf1, zk1, 'M');
    end
    % SINS/CNS feterated sub-KF2
    kf2.Pxk = fkf.Pxk;  kf2.xk = fkf.xk;  % sub-KF2 reset
    kf2.Phikk_1 = kffk(fins);
    kf2 = kfupdate(kf2);
    if mod(t,1)<1.5*ts && norm(fins.wnb)<1*glv.dps
        kf2.Hk(2:3,8) = -[fins.eth.cl; fins.eth.sl];
        Cns = cnsCns(qis(k1,1:3)', fins.pos, Cie0, t);
        zk2 = qq2phi(fins.qnb,m2qua(Cns));
        kf2 = kfupdate(kf2, zk2, 'M');
    end
    % KF master
    if fkf.fbeta>0
        fkf.Phikk_1 = kffk(fins);
        fkf = kfupdate(fkf);
    end
    % feterated KF fusion
    iP1 = invbc( kf1.Pxk );   iP2 = invbc( kf2.Pxk );
    if fkf.fbeta>0, iPm = invbc( fkf.Pxk );  else iPm = fkf.Pxk*0; end
	fkf.Pxk = invbc( iP1 + iP2 + iPm );
	fkf.xk = fkf.Pxk * (iP1*kf1.xk + iP2*kf2.xk + iPm*fkf.xk);
    [fkf, fins] = kffeedback(fkf, fins, 1, 'avp');
    % save results
    avp(ki,:)   = [ins.avp', t];
    xkpk(ki,:)  = [kf.xk; diag(kf.Pxk); t]';
	xkpk1(ki,:) = [kf1.xk; diag(kf1.Pxk); t]';
	xkpk2(ki,:) = [kf2.xk; diag(kf2.Pxk); t]';
    xkpkf(ki,:) = [fkf.xk; diag(fkf.Pxk); t]';
    ki = timebar;
end
[avp,xkpk,xkpk1,xkpk2] = no0s(avp,xkpk,xkpk1,xkpk2);
insplot(avp);
avpcmpplot(avp, trj.avp);
kfplot(xkpk);
kfplot(xkpkf);
% kfplot(xkpk1);
% kfplot(xkpk2);

