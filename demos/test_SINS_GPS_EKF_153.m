% SINS/GPS intergrated navigation simulation using EKF.
% Please run 'test_SINS_trj.m' to generate 'trj10ms.mat' beforehand!!!
% See also  test_SINS_GPS_UKF_153, test_SINS_GPS_CKF_153, test_SINS_GPS_153.
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/02/2022
glvs
psinstypedef('test_SINS_GPS_EKF_153_def');
trj = trjfile('trj10ms.mat');  % insplot(trj.avp);
% initial settings
[nn, ts, nts] = nnts(2, trj.ts);
imuerr = imuerrset(0.03, 100, 0.001, 5);
imu = imuadderr(trj.imu, imuerr);  % imuplot(imu);
davp0 = avperrset([1;1;10]*60, 0.1, [1;1;3]);
ins = insinit(avpadderr(trj.avp0,davp0), ts);
% CKF filter
rk = poserrset([1;1;3]);
kf = kfinit(ins, davp0, imuerr, rk);
len = length(imu); [avp, xkpk] = prealloc(fix(len/nn), 10, 2*kf.n+1);
timebar(nn, len, '15-state SINS/GPS EKF Simulation.'); 
ki = 1;
for k=1:nn:len-nn+1
    k1 = k+nn-1;  
    wvm = imu(k:k1,1:6);  t = imu(k1,end);
    ins = insupdate(ins, wvm);
    kf.px = ins;
	kf = ekf(kf);
    if mod(t,1)==0
        posGPS = trj.avp(k1,7:9)' + davp0(7:9).*randn(3,1);  % GPS pos simulation with some white noise
        kf = ekf(kf, ins.pos-posGPS, 'M');  % EKF filter
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

