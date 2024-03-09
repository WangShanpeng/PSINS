% Comparision of EKF & SE(3) SINS/GPS intergrated navigation simulation, with
% 15-state included:
%       phi(3), dvn(3), dpos(3), eb(3), db(3)
% Please run 'test_SINS_trj.m' to generate 'trj10ms.mat' beforehand!!!
% See also  test_SINS_GPS_153, test_SINS_GPS_156_STEKF.
% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 13/02/2023
glvs
psinstypedef(156);
trj = trjfile('trj10ms.mat');  % insplot(trj.avp)
[nn, ts, nts] = nnts(2, trj.ts);
%% init
imuerr = imuerrset(0.03, 100, 0.001, 10);
imu = imuadderr(trj.imu, imuerr);
davp0 = avperrset([1.0*60;-1.0*60;160.0*60], [0.1;0.1;0.1], [1;1;3]);
gps = gpssimu(trj.avp, davp0(4:6), davp0(7:9), 1);
imugpssyn(imu(:,7), gps(:,7));
ins = insinit(trj.avp0(1:9), ts, davp0);
%% kf
r0 = vperrset(0.1, 1);
kf = kfinit(ins, davp0, imuerr, r0);   % kf.Pxk(10:15,10:15)=0;
len = length(imu); [avp, xkpk] = prealloc(fix(len/nn), 10, 2*kf.n+1);
timebar(nn, len, '15-state SINS/GPS EKF & SE(3) comparision simulation.'); ki = 1;
ins_se = ins; kf_se = kf; avp_se = avp; xkpk_se = xkpk;  % SE(3)
for k=1:nn:len-nn+1
    k1 = k+nn-1; 
    wvm = imu(k:k1,1:6); t = imu(k1,end);
    ins = insupdate(ins, wvm);
    kf.Phikk_1 = kffk(ins);
    kf = kfupdate(kf);
    ins_se = insupdate(ins_se, wvm);  kf_se.Phikk_1 = eye(15)+etm_LSE(ins_se)*nts;  kf_se = kfupdate(kf_se);  % SE(3)
    [kgps, dt] = imugpssyn(k, k1, 'F');
    if kgps>0
        vnGPS = gps(kgps,1:3)'; posGPS = gps(kgps,4:6)';
        kf.Hk = kfhk(ins);    kf_se.Hk(4:6,7:9) = -ins.Cnb*0;
        zk = [ins.vn-vnGPS; ins.pos-posGPS];
        kf = kfupdate(kf, zk, 'M');
        ins.pos = ins.pos-kf.xk(7:9);  kf.xk(7:9) = 0;
        ins.vn = ins.vn-kf.xk(4:6);  kf.xk(4:6) = 0;
        ins.qnb = qdelphi(ins.qnb, kf.xk(1:3)); kf.xk(1:3) = 0;
        avp(ki,:) = [ins.avp', t];
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';
        kf_se.Hk = kf.Hk;  kf_se.Hk(1:3,4:6) = -ins.Cnb;  % SE(3)
        zk_se = [ins_se.vn-vnGPS; ins_se.pos-posGPS];
        kf_se = kfupdate(kf_se, zk_se, 'M');
        ins_se.pos = ins_se.pos-kf_se.xk(7:9);  kf_se.xk(7:9) = 0;
        ins_se.vn = ins_se.vn+ins.Cnb*kf_se.xk(4:6);  kf_se.xk(4:6) = 0;
        ins_se.qnb = qmul(ins_se.qnb, rv2q(kf_se.xk(1:3))); kf_se.xk(1:3) = 0;
        avp_se(ki,:) = [ins_se.avp', t];
        xkpk_se(ki,:) = [kf_se.xk; diag(kf_se.Pxk); t]';
        ki = ki+1;
    end
    timebar;
end
avp(ki:end,:) = []; xkpk(ki:end,:) = [];  avp_se(ki:end,:) = []; xkpk_se(ki:end,:) = [];
avperr = avpcmpplot(trj.avp, avp);
kfplot(xkpk, avperr, imuerr);
avperr_se = avpcmpplot(trj.avp, avp_se);
kfplot(xkpk_se, avperr_se, imuerr);
inserrplot(avperr);
err = avperr_se; t = err(:,end);
subplot(221), plot(t, err(:,1:2)/glv.sec, 'm');  subplot(222), plot(t, err(:,3)/glv.min, 'm');  legend('\it\phi\rm_{U,EKF}', '\it\phi\rm_{U,SE(3)}');
subplot(223), plot(t, err(:,4:6), 'm');  subplot(224), plot(t, [err(:,7:8)*glv.Re,err(:,9)], 'm');




