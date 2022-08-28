% SINS/GPS intergrated navigation simulation using M-estimator Kalman filter.
% Please run 'test_SINS_trj.m' to generate 'trj10ms.mat' beforehand!!!
% See also  test_SINS_GPS_153.
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/06/2022
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
timebar(nn, len, '15-state SINS/GPS M-Estimator Kalman filter Simulation.'); 
ki = 1;
for k=1:nn:len-nn+1
    k1 = k+nn-1;  
    wvm = imu(k:k1,1:6);  t = imu(k1,end);
    ins = insupdate(ins, wvm);
    kf.Phikk_1 = kffk(ins);
    kf = kfupdate(kf);
    if mod(t,1)==0
        if t<400, s=1; elseif t<800, s=100; else s=1; end
        posGPS = trj.avp(k1,7:9)' + davp0(7:9).*htwn(0.1,s,3);  % GPS pos simulation with some heavy-tailed noise
        zk = ins.pos-posGPS;    rk = zk-kf.Hk*kf.xk;
        w = igg1(rk./sqrt(diag(kf.Pykk_1)),3,inf);
        kf = kfupdate(kf, w.*zk, 'M');
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

