% SINS/GPS intergrated navigation simulation using PF(particle filter).
% Please run 'test_SINS_trj.m' to generate 'trj10ms.mat' beforehand!!!
% See also  test_SINS_GPS_UKF_153, test_SINS_GPS_EKF_153, test_SINS_GPS_153.
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/04/2022
%   NOTE: 暂且运行不对！
glvs
psinstypedef('test_SINS_GPS_PF_153_def');
trj = trjfile('trj10ms.mat');  % insplot(trj.avp);
% initial settings
[nn, ts, nts] = nnts(2, trj.ts);
imuerr = imuerrset(0.03, 100, 0.001, 5);
imu = imuadderr(trj.imu, imuerr);  % imuplot(imu);
davp0 = avperrset([1;1;10]*60, 0.1, [1;1;3]);
ins = insinit(avpadderr(trj.avp0,davp0), ts);
% PF filter
rk = poserrset([1;1;3]);
pf = kfinit(ins, davp0, imuerr, rk, 500);
len = length(imu); [avp, xkpk] = prealloc(fix(len/nn), 10, 2*pf.n+1);
timebar(nn, len, '15-state SINS/GPS PF Simulation.'); 
ki = 1;
for k=1:nn:len/10-nn+1
    k1 = k+nn-1;  
    wvm = imu(k:k1,1:6);  t = imu(k1,end);
    ins = insupdate(ins, wvm);
    pf.tpara = ins;
	pf = pfupdate(pf);
    if mod(t,1)==0
        posGPS = trj.avp(k1,7:9)' + davp0(7:9).*randn(3,1);  % GPS pos simulation with some white noise
        pf = pfupdate(pf, ins.pos-posGPS, 'M');  % PF filter
        [pf, ins, xfb] = kffeedback(pf, ins, 1, 'avp');
        pf.particles = pf.particles - repmat(xfb,1,pf.Npts);
        avp(ki,:) = [ins.avp', t];
        xkpk(ki,:) = [pf.xk; diag(pf.Pxk); t]';  ki = ki+1;
    end
    timebar;
end
avp(ki:end,:) = [];  xkpk(ki:end,:) = []; 
% show results
insplot(avp);
avperr = avpcmpplot(trj.avp, avp);
kfplot(xkpk, avperr, imuerr);

