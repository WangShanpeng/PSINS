% SINS/GPS intergrated navigation simulation using StrongTracking KF. (trash!!!)
% Please run 'test_SINS_trj.m' to generate 'trj10ms.mat' beforehand!!!
% See also  test_SINS_trj, test_SINS, test_SINS_GPS_153.
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/02/2023
glvs
psinstypedef(153);
trj = trjfile('trj10ms.mat');
% initial settings
[nn, ts, nts] = nnts(2, trj.ts);
imuerr = imuerrset(0.03, 100, 0.001, 5);
imu = imuadderr(trj.imu, imuerr);
davp0 = avperrset([0.5;-0.5;20], 0.1, [1;1;3]);
ins = insinit(avpadderr(trj.avp0,davp0), ts);
% KF filter
rk = poserrset([1;1;3]);
kf = kfinit(ins, davp0, imuerr, rk);
kf.Pmin = [avperrset(0.01,1e-4,0.1); gabias(1e-3, [1,10])].^2;  kf.pconstrain=1;
len = length(imu); [avp, xkpk] = prealloc(fix(len/nn), 10, 2*kf.n+1);
timebar(nn, len, '15-state SINS/GPS  StrongTracking KF Simulation.'); 
ki = 1;
b = 0.95;  betak = 1.0;  Ck = 100*kf.Rk;
for k=1:nn:len-nn+1
    k1 = k+nn-1;  
    wvm = imu(k:k1,1:6);  t = imu(k1,end);
    ins = insupdate(ins, wvm);
    kf.Phikk_1 = kffk(ins);
    kf = kfupdate(kf);
    if mod(t,1)==0
        posGPS = trj.avp(k1,7:9)' + davp0(7:9).*randn(3,1);  % GPS pos simulation with some white noise        
        Zk = ins.pos-posGPS;  % ---------------------------- StrongTracking(trash!!!)
        Ck = (1-betak)*Ck+betak*Zk*Zk';  betak = betak/(betak+b);   
        Mk = kf.Hk*kf.Phikk_1*kf.Pxk*kf.Phikk_1'*kf.Hk';
        Nk = Ck-kf.Hk*kf.Gammak*kf.Qk*kf.Gammak'*kf.Hk'-kf.Rk;
        kf.fading = max(1,trace(Nk)/trace(Mk));  % --------- StrongTracking
%         kf.fading = 1;  % good
        kf = kfupdate(kf, ins.pos-posGPS, 'M');
        [kf, ins] = kffeedback(kf, ins, 1, 'avp');
        avp(ki,:) = [ins.avp', t];
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';  ki = ki+1;
    end
    timebar;
end
avp(ki:end,:) = [];  xkpk(ki:end,:) = []; 
% show results
insplot(avp);
avperr = avpcmpplot(trj.avp, avp);
kfplot(xkpk, avperr, imuerr);

