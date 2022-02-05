% SINS/GPS/CNS centralized v.s. federated KF with 21-state include:
%       phi(3), dvn(3), dpos(3), eb(3), db(3), lv(3), mu(3)
% Please run 'test_SINS_trj.m' to generate 'trj10ms.mat' beforehand!!!
% See also  sinsgps, test_SINS_GPS_186, test_SINS_CNS_184.
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/01/2022
glvs
trj = trjfile('trj10ms.mat');
[nn, ts, nts] = nnts(2, trj.ts);
% GPS simulator
lever = [1; 2; 3]*0; rk1 = vperrset(0.1, 1);
davp0 = avperrset([0.5;-0.5;20], 0.01, [1;1;3]);
gps = gpssimu(trj.avp, davp0(4:6), davp0(7:9), 1, lever, 0.0);  % gpsplot(gps)
% CNS simulator
mu = [1;2;5]*glv.min;  rk2 = [[10;10;30]*glv.sec];
% [qis, utc0] = cnssimu(trj.avp, rk2, mu, [2021;11;22;12*3600; -0.1;37]);
% SINS init
imuerr = imuerrset(0.03, [100;100;100], 0.001, 5);
imu = imuadderr(trj.imu, imuerr);  % imuplot(imu)
ins = insinit(trj.avp0(1:9), ts, davp0);  len = length(imu);
Cie0 = cnsCie(utc0(1:3), utc0(4),  utc0(5), utc0(6));
% centralized KF
psinstypedef('test_SINS_GPS_CNS_def');
kf = kfinit(ins, davp0, imuerr, [rk1;rk2], lever, mu);
kf = kfinit0(kf, nts);
% SINS/GPS KF1
kf1 = [];
kf1.fbeta = 1/2; kf1.nfkf = 15; kf1.pfkf = 3;
kf1.Qt = kf.Qt(1:18,1:18);  kf1.Pxk = kf.Pxk(1:18,1:18);
kf1.Rk = diag(rk1)^2;
kf1.Hk = kf.Hk(1:6,1:18);
kf1 = kfinit0(kf1, nts);
% SINS/CNS KF2
kf2 = [];
kf2.fbeta = 1/3; kf2.nfkf = 15; kf2.pfkf = 3;
kf2.Qt = kf.Qt([1:15,19:21],[1:15,19:21]);   kf2.Pxk = kf.Pxk([1:15,19:21],[1:15,19:21]);
kf2.Rk = diag(rk2)^2;
kf2.Hk = kf.Hk(7:9,[1:15,19:21]);
kf2 = kfinit0(kf2, nts);
% feterated KF for SINS/GPS & SINS/CNS
fins = ins;
fkf = [];
fkf.fbeta = 1-kf1.fbeta-kf2.fbeta;  fkf.nfkf = 15;  fkf.pfkf = 3;
if fkf.fbeta<0 || fkf.fbeta>1, error('FKF info-factor must be within [0,1] !'); end
fkf.Qt = kf.Qt(1:15,1:15);   fkf.Pxk = kf.Pxk(1:15,1:15);
fkf.Rk = inf(1);
fkf.Hk = zeros(1,15);
fkf = kfinit0(fkf, nts);
% 
[avp,  xkpk]  = prealloc(fix(len/nn), 10, 2*kf.n+1);
[avp1, xkpk1] = prealloc(fix(len/nn), 10, 2*kf1.n+1);
[avp2, xkpk2] = prealloc(fix(len/nn), 10, 2*kf2.n+1);
[avpf, xkpkf] = prealloc(fix(len/nn), 10, 2*fkf.n+1);
ki = timebar(nn, len, '21-state SINS/GPS/CNS centralized v.s. federated KF simulation.');
imugpssyn(imu(:,7), gps(:,end)); 
for k=1:nn:len-nn+1
    k1 = k+nn-1;
    wvm = imu(k:k1,1:6); t = imu(k1,end);
    %% SINS/GPS/CNS sequential centralized KF
    ins = insupdate(ins, wvm);
    kf.Phikk_1 = kffk(ins);
    kf = kfupdate(kf);
    [kgps, dt] = imugpssyn(k, k1, 'F');
    if kgps>0
        ins = inslever(ins);
        zk1 = [ins.vnL-gps(kgps,1:3)'; ins.posL-gps(kgps,4:6)'];
        Hk1 = [zeros(6,3), eye(6), zeros(6,6), [-ins.CW;-ins.MpvCnb], zeros(6,3)];
        kf.Hk = Hk1;  kf.Rk = kf1.Rk;   kf = kfupdate(kf, zk1, 'M');
    end
    if mod(t,1)<1.5*ts && norm(ins.wnb)<1*glv.dps
        Hk2 = [eye(3), zeros(3,15), ins.Cnb];  Hk2(1,7)=1;   Hk2(2:3,8) = -[ins.eth.cl; ins.eth.sl];
        Cns = cnsCns(qis(k1,1:3)', ins.pos, Cie0, t);
        zk2 = qq2phi(ins.qnb,m2qua(Cns));
        kf.Hk = Hk2;  kf.Rk = kf2.Rk;   kf = kfupdate(kf, zk2, 'M');
    end
    [kf, ins] = kffeedback(kf, ins, 1, 'avp');
    %% SINS/GPS feterated sub-KF1
    fins = insupdate(fins, wvm);
    kf1.Pxk = psetfkf(kf1.Pxk, fkf.Pxk);  kf1.xk(1:kf1.nfkf) = fkf.xk;  % sub-KF1 reset
    kf1.fbeta = 1/2;
    kf1.Phikk_1 = kf.Phikk_1(1:18,1:18);
    kf1 = kfupdate(kf1);
    if kgps>0
        fins = inslever(fins);
        zk1 = [fins.vnL-gps(kgps,1:3)'; fins.posL-gps(kgps,4:6)'];
        kf1.Hk = Hk1(:,1:18);
        kf1 = kfupdate(kf1, zk1, 'M');
    end
    % SINS/CNS feterated sub-KF2
    kf2.Pxk = psetfkf(kf2.Pxk, fkf.Pxk);  kf2.xk(1:kf2.nfkf) = fkf.xk;  % sub-KF2 reset
    kf2.fbeta = 1/2;
    kf2.Phikk_1 = kf.Phikk_1([1:15,19:21],[1:15,19:21]);
    kf2 = kfupdate(kf2);
    if mod(t,1)<1.5*ts && norm(fins.wnb)<1*glv.dps
        kf2.Hk = Hk2(:,[1:15,19:21]);
        Cns = cnsCns(qis(k1,1:3)', fins.pos, Cie0, t);
        zk2 = qq2phi(fins.qnb,m2qua(Cns));
        kf2 = kfupdate(kf2, zk2, 'M');
    end
    % KF master
    fkf.fbeta = 0;
    if fkf.fbeta>0
        fkf.Phikk_1 = kf.Phikk_1(1:15,1:15);
        fkf = kfupdate(fkf);
    end
    % feterated KF fusion
    iP1 = invbc( kf1.Pxk(1:kf1.nfkf,1:kf1.nfkf) );   iP2 = invbc( kf2.Pxk(1:kf2.nfkf,1:kf2.nfkf) );
    if fkf.fbeta>0, iPm = invbc( fkf.Pxk );  else iPm = fkf.Pxk*0; end
	fkf.Pxk = invbc( iP1 + iP2 + iPm );
	fkf.xk = fkf.Pxk * (iP1*kf1.xk(1:kf1.nfkf) + iP2*kf2.xk(1:kf2.nfkf) + iPm*fkf.xk);
    [fkf, fins, xfb] = kffeedback(fkf, fins, 1, 'avp');
%     kf1.xk(1:kf1.nfkf) = kf1.xk(1:kf1.nfkf)-xfb;  kf2.xk(1:kf2.nfkf) = kf2.xk(1:kf2.nfkf)-xfb;
    %% save results
    avp(ki,:)   = [ins.avp', t];
    avpf(ki,:)  = [fins.avp', t];
    xkpk(ki,:)  = [kf.xk; diag(kf.Pxk); t]';
	xkpk1(ki,:) = [kf1.xk; diag(kf1.Pxk); t]';
	xkpk2(ki,:) = [kf2.xk; diag(kf2.Pxk); t]';
    xkpkf(ki,:) = [fkf.xk; diag(fkf.Pxk); t]';
    ki = timebar;
end
[avp,xkpk,xkpk1,xkpk2,avpf] = no0s(avp,xkpk,xkpk1,xkpk2,avpf);
% insplot(avp);
% avpcmpplot(avp, trj.avp); % avpcmpplot(avpf, trj.avp);
inserrplot(xkpk(:,[1:18,end]));  subplot(428), plot(xkpk(:,end), xkpk(:,19:21)/glv.min); xygo('mu');  % insserrplot(xkpk(:,[22:39,end]));
inserrplot(xkpk1(:,[1:18,end]));
inserrplot(xkpk2(:,[1:18,end]));  subplot(427), hold off; plot(xkpk2(:,end), xkpk2(:,16:18)/glv.min); xygo('mu');
% inserrplot(xkpkf(:,[1:15,end]));
return;
% 
insserrplot(xkpk(:,[22:39,end])); subplot(428), plot(xkpk(:,end), sqrt(xkpk(:,40:42))/glv.min); xygo('mu');
insserrplot(xkpk1(:,[19:36,end])); subplot(428), plot(xkpk2(:,end), sqrt(xkpk2(:,34:36))/glv.min); xygo('mu');


