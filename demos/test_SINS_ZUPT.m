% SINS ZUPT using 15-state Kalman filter.
% Please run 'test_SINS_ZUPT.m' to generate 'trj10ms.mat' beforehand!!!
% See also  test_SINS, test_SINS_GPS_153.
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/02/2022
glvs
psinstypedef(156);
trj = trjfile('trj10ms.mat');
% initial settings
[nn, ts, nts] = nnts(2, trj.ts);
imuerr = imuerrset(0.01, 100, 0.001, 5);
imu = imuadderr(trj.imu, imuerr);
davp0 = avperrset([0.5;-0.5;5], 0.001, [1;1;3]);
ins = insinit(avpadderr(trj.avp0,davp0), ts);
% ZUPT KF filter
rk = vperrset([1;1;3]*0.01, [1;1;1]);
kf = kfinit(ins, davp0, imuerr, rk);  kf.Hk(4:6,:) = 0; % no pos meas
kf.Pmin = [avperrset(0.01,1e-4,0.1); gabias(1e-3, [1,10])].^2;  kf.pconstrain=1;
len = length(imu); [avp, xkpk] = prealloc(fix(len/nn), 10, 2*kf.n+1);
ki = timebar(nn, len, 'SINS ZUPT Simulation.'); 
for k=1:nn:len-nn+1
    k1 = k+nn-1;  
    wvm = imu(k:k1,1:6);  t = imu(k1,end);
    ins = insupdate(ins, wvm);
    kf.Phikk_1 = kffk(ins);
    kf = kfupdate(kf);
    if mod(t,1)==0 && mod(t,600)<10
        kf = kfupdate(kf, [ins.vn; zeros(3,1)], 'M');
        [kf, ins] = kffeedback(kf, ins, 1, 'avped');
    end
	avp(ki,:) = [ins.avp', t];
	xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';
    ki = timebar;
end
avp(ki:end,:) = [];  xkpk(ki:end,:) = []; 
% show results
insplot(avp);
avperr = avpcmpplot(trj.avp, avp);
kfplot(xkpk, avperr, imuerr);

