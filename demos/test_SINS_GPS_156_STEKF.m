% Comparision of EKF & STEKF SINS/GPS intergrated navigation simulation, with
% 15-state included:
%       phi(3), dvn(3), dpos(3), eb(3), db(3)
% Please run 'test_SINS_trj.m' to generate 'trj10ms.mat' beforehand!!!
% See also  test_SINS_GPS_153, test_SINS_GPS_153_SE3.
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/08/2022
glvs
psinstypedef(156);
trj = trjfile('trj10ms.mat');  % insplot(trj.avp)
[nn, ts, nts] = nnts(2, trj.ts);
%% init
imuerr = imuerrset(0.03, 100, 0.001, 10);
imu = imuadderr(trj.imu, imuerr);
davp0 = avperrset([60;-60;60.0*60], [0.1;0.1;0.1], [1;1;3]);
gps = gpssimu(trj.avp, davp0(4:6), davp0(7:9), 1);
imugpssyn(imu(:,7), gps(:,7));
ins = insinit(trj.avp0(1:9), ts, davp0);
%% kf
r0 = vperrset(0.1, 1);
kf = kfinit(ins, davp0, imuerr, r0);
len = length(imu); [avp, xkpk] = prealloc(fix(len/nn), 10, 2*kf.n+1);
timebar(nn, len, '15-state SINS/GPS EKF & STEKF comparision simulation.'); ki = 1;
ins_st = ins; kf_st = kf; avp_st = avp; xkpk_st = xkpk;  % ST
A = eye(kf_st.n); A(4:6,1:3)=-askew(ins_st.vn);  kf_st.Pxk=A*kf_st.Pxk*A';
for k=1:nn:len-nn+1
    k1 = k+nn-1; 
    wvm = imu(k:k1,1:6); t = imu(k1,end);
    ins = insupdate(ins, wvm);
    kf.Phikk_1 = kffk(ins);
    kf = kfupdate(kf);
    ins_st = insupdate(ins_st, wvm);  kf_st.Phikk_1 = eye(15)+etmST(ins_st)*nts;  kf_st = kfupdate(kf_st);  % ST
    [kgps, dt] = imugpssyn(k, k1, 'F');
    if kgps>0
        vnGPS = gps(kgps,1:3)'; posGPS = gps(kgps,4:6)';
        kf.Hk = kfhk(ins);
        zk = [ins.vn-vnGPS; ins.pos-posGPS];
        kf = kfupdate(kf, zk, 'M');
        ins.pos = ins.pos-kf.xk(7:9);  kf.xk(7:9) = 0;
        ins.vn = ins.vn-kf.xk(4:6);  kf.xk(4:6) = 0;
        ins.qnb = qdelphi(ins.qnb, kf.xk(1:3)); kf.xk(1:3) = 0;
        avp(ki,:) = [ins.avp', t];
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';
        kf_st.Hk = kf.Hk;  kf_st.Hk(1:3,1:3) = askew(ins_st.vn);  % ST
        zk_st = [ins_st.vn-vnGPS; ins_st.pos-posGPS];
        kf_st = kfupdate(kf_st, zk_st, 'M');
        ins_st.pos = ins_st.pos-kf_st.xk(7:9);  kf_st.xk(7:9) = 0;
        dv = kf_st.xk(4:6)+askew(ins_st.vn)*kf_st.xk(1:3);   ins_st.vn = ins_st.vn-dv;  kf_st.xk(4:6) = 0;
        ins_st.qnb = qdelphi(ins_st.qnb, kf_st.xk(1:3)); kf_st.xk(1:3) = 0;
        avp_st(ki,:) = [ins_st.avp', t];
        xkpk_st(ki,:) = [kf_st.xk; diag(kf_st.Pxk); t]';
        ki = ki+1;
    end
    timebar;
end
avp(ki:end,:) = []; xkpk(ki:end,:) = [];  avp_st(ki:end,:) = []; xkpk_st(ki:end,:) = [];
avperr = avpcmpplot(trj.avp, avp);
kfplot(xkpk, avperr, imuerr);
avperr_st = avpcmpplot(trj.avp, avp_st);
kfplot(xkpk_st, avperr_st, imuerr);
inserrplot(avperr);
err = avperr_st; t = err(:,end);
subplot(221), plot(t, err(:,1:2)/glv.sec, 'm');  subplot(222), plot(t, err(:,3)/glv.min, 'm');  legend('\it\phi\rm_{U,EKF}', '\it\phi\rm_{U,STEKF}');
subplot(223), plot(t, err(:,4:6), 'm');  subplot(224), plot(t, [err(:,7:8)*glv.Re,err(:,9)], 'm');




