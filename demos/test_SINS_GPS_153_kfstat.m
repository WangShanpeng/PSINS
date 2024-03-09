% Kalman filter error distribution analysis and statistic for SINS/GPS.
% Please run 'test_SINS_trj.m' to generate 'trj10ms.mat' beforehand!!!
% See also  test_SINS_GPS_153.
% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/11/2023
glvs
psinstypedef(153);
trj = trjfile('trj10ms.mat');
% initial settings
[nn, ts, nts] = nnts(2, trj.ts);
imuerr = imuerrset(0.3, 1000, .01, 50);
imu = imuadderr(trj.imu, imuerr);
davp0 = avperrset([0.5;-0.5;20], 0.1, [1;1;3]);
ins = insinit(avpadderr(trj.avp0,davp0), ts);
% KF filter
rk = poserrset([1;1;3]*10);
kf = kfinit(ins, davp0, imuerr, rk);
kf.Pmin = [avperrset(0.01,1e-4,0.1); gabias(1e-3, [1,10])].^2;  kf.pconstrain=0;
kfs = kfstat([], kf);
len = length(imu); [avp, xkpk] = prealloc(fix(len/nn), 10, 2*kf.n+1);
pqr = zeros(kf.n, 2*kf.n+kf.m, fix(len/nn));
timebar(nn, len, '15-state SINS/GPS Simulation.'); 
ki = 1;
for k=1:nn:len-nn+1
    k1 = k+nn-1;  
    wvm = imu(k:k1,1:6);  t = imu(k1,end);
    ins = insupdate(ins, wvm);
    kf.Phikk_1 = kffk(ins);
    kf = kfupdate(kf);
    kfs = kfstat(kfs, kf, 'T');
    if mod(t,1)==0
        posGPS = trj.avp(k1,7:9)' + davp0(7:9).*randn(3,1);  % GPS pos simulation with some white noise
        kf = kfupdate(kf, ins.pos-posGPS, 'M');
        kfs = kfstat(kfs, kf, 'M');
        kfs = kfstat(kfs);
        pqr(:,:,ki) = [kfs.p, kfs.q, kfs.r];
        [kf, ins] = kffeedback(kf, ins, 1, 'avp');
        avp(ki,:) = [ins.avp', t];
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';  ki = ki+1;
    end
    timebar;
end
avp(ki:end,:) = [];  xkpk(ki:end,:) = [];  pqr(:,:,ki:end) = [];
%% show results
% insplot(avp);
% avperr = avpcmpplot(trj.avp, avp);
% kfplot(xkpk, avperr, imuerr);
myfig
pqri = pqr(1:3,:,end); subplot(321), plot(pqri'*100, '-.o'); xygo('k', 'phi %');
pqri = pqr(4:6,:,end); subplot(323), plot(pqri'*100, '-.o'); xygo('k', 'dV %');
pqri = pqr(7:9,:,end); subplot(325), plot(pqri'*100, '-.o'); xygo('k', 'dP %');
pqri = pqr(10:12,:,end); subplot(322), plot(pqri'*100, '-.o'); xygo('k', 'eb %');
pqri = pqr(13:15,:,end); subplot(324), plot(pqri'*100, '-.o'); xygo('k', 'db %');
%
pqr1 = permute(pqr,[3,2,1]);
myfig,
for k=1:15, subplot(3,5,k); plot(pqr1(:,:,k)); textmax(pqr1(:,:,k));  end
%
h0 = figure; set(h0,'Visible','off');
for k=1:15
    h=figure(123+k); set(h, 'WindowStyle','docked','NumberTitle','off','Name',num2str(k));
    plot(pqr1(:,:,k)); textmax(pqr1(:,:,k)); xygo(['x _{',num2str(k),'}']);
end
close(h0);
