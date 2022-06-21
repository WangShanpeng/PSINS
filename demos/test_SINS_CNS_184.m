% SINS/CNS intergrated navigation simulation using Kalman filter.
% Please run 'test_SINS_trj.m' to generate 'trj10ms.mat' beforehand!!!
% See also  test_SINS_trj, test_SINS, test_SINS_GPS_153.
% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/11/2021
glvs
psinstypedef('test_SINS_CNS_def');
trj = trjfile('trj10ms.mat');
%% CNS simulation
mu = [1;2;5]*glv.min;
rk = [[10;10;30]*glv.sec;10];
[qis, utc0] = cnssimu(trj.avp, rk(1:3), mu, [2021;11;22;12*3600; -0.1;37]);
%% initial settings
[nn, ts, nts] = nnts(1, trj.ts);
imuerr = imuerrset(0.03, 100, 0.001, .5);
imu = imuadderr(trj.imu, imuerr);
davp0 = avperrset([0.5;-0.5;20], 0.01, [1;1;3]);
ins = insinit(avpadderr(trj.avp0, davp0), ts);
%% KF filter
Cie0 = cnsCie(utc0(1:3), utc0(4),  utc0(5), utc0(6));
kf = kfinit(ins, davp0, imuerr, rk, mu);
len = length(imu); [avp, xkpk] = prealloc(fix(len/nn), 10, 2*kf.n+1);
timebar(nn, len, '18-state SINS/CNS Simulation.'); 
Cbs = eye(3);
ki = 1;
for k=1:nn:len-nn+1
    k1 = k+nn-1;  
    wvm = imu(k:k1,1:6);  t = imu(k1,end);
    ins = insupdate(ins, wvm);
    kf.Phikk_1 = kffk(ins);
    kf = kfupdate(kf);
    if mod(t,1)==0 & abs(ins.wnb)<1*glv.dps
        kf.Hk = kfhk(ins);
        Cns = cnsCns(qis(k1,1:3)', ins.pos, Cie0, t, Cbs);
        zk = [qq2phi(ins.qnb,m2qua(Cns));ins.pos(3)-trj.avp(k1,9)+rk(4)*randn(1)];
        kf = kfupdate(kf, zk, 'M');
        [kf, ins] = kffeedback(kf, ins, 1, 'avp');
%         Cbs = Cbs*a2mat(kf.xk(16:18));  kf.xk(16:18)=0;  % mu feedback
    end
    avp(ki,:) = [ins.avp', t];
    xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';  ki = ki+1;
    timebar;
end
avp(ki:end,:) = [];  xkpk(ki:end,:) = []; 
%% show results
insplot(avp);
avpcmpplot(avp, trj.avp);
kfplot(xkpk);
