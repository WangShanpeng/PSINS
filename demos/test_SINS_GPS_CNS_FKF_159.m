% SINS/GPS/CNS centralized v.s. federated KF with 15-state include:
%       phi(3), dvn(3), dpos(3), eb(3), db(3)
% Please run 'test_SINS_trj.m' to generate 'trj10ms.mat' beforehand!!!
% See also  sinsgps, test_SINS_GPS_156, test_SINS_CNS_184.
% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/12/2021
glvs
trj = trjfile('trj10ms.mat');
[nn, ts, nts] = nnts(2, trj.ts);
% GPS simulator
lever = [1; 2; 3]*0; rk1 = vperrset(0.1, 1);
davp0 = avperrset([0.5;-0.5;20], 0.01, [1;1;3]);
gps = gpssimu(trj.avp, davp0(4:6), davp0(7:9), 1, lever, 0.0);  % gpsplot(gps)
% CNS simulator
mu = 0*[1;2;5]*glv.min;  rk2 = [[10;10;30]*glv.sec];
[qis, utc0] = cnssimu(trj.avp, rk2, mu, [2021;11;22;12*3600; -0.1;37]);
% SINS init
imuerr = imuerrset(0.03, [100;100;100], 0.001, 5);
imu = imuadderr(trj.imu, imuerr);  % imuplot(imu)
ins = insinit(trj.avp0(1:9), ts, davp0);  len = length(imu);
Cie0 = cnsCie(utc0(1:3), utc0(4),  utc0(5), utc0(6));
% centralized KF setting
psinstypedef(156);
ckf = [];
ckf.Phikk_1 = eye(15); ckf.Qt = diag([imuerr.web; imuerr.wdb; zeros(9,1)])^2;  % 15-state
ckf.Rk = diag([rk1; rk2])^2;
ckf.Pxk = diag([davp0; imuerr.eb; imuerr.db]*1.0)^2;
ckf.Hk = [ zeros(6,3), eye(6), zeros(6,6); ...    % SINS/GPS Hk
          eye(3), zeros(3,12) ];  ckf.Hk(7,7)=1;  % SINS/CNS Hk
ckf = kfinit0(ckf, nts); 
% federated KF setting
fins = ins;
fkf = fkfinit(ckf, {1:15,1:15}, {1:6,7:9}, [1/2; 1/3]);
% alloc memory
[avpc, xkpkc] = prealloc(fix(len/nn), 10, 2*ckf.n+1);
[avp1, xkpk1] = prealloc(fix(len/nn), 10, 2*fkf{1}.n+1);
[avp2, xkpk2] = prealloc(fix(len/nn), 10, 2*fkf{2}.n+1);
[avpf, xkpkf] = prealloc(fix(len/nn), 10, 2*fkf{3}.n+1);
ki = timebar(nn, len, '15-state SINS/GPS/CNS centralized v.s. federated KF simulation.');
imugpssyn(imu(:,7), gps(:,end)); 
for k=1:nn:len-nn+1
    k1 = k+nn-1;
    wvm = imu(k:k1,1:6); t = imu(k1,end);
    ins = insupdate(ins, wvm);
    fins = insupdate(fins, wvm);
    ckf.Phikk_1 = kffk(ins);
    [kgps, dt] = imugpssyn(k, k1, 'F');
    if kgps>0 && mod(t,1)<1.5*ts && norm(ins.wnb)<1*glv.dps  % SINS/GPS/CNS
        Cns = cnsCns(qis(k1,1:3)', ins.pos, Cie0, t);
        ckf.Hk(8:9,8) = -[ins.eth.cl; ins.eth.sl];
        zk = [ins.vn-gps(kgps,1:3)'; ins.pos-gps(kgps,4:6)'; qq2phi(ins.qnb,m2qua(Cns))];
        ckf = kfupdate(ckf, zk);  % centralized KF
        Cns = cnsCns(qis(k1,1:3)', fins.pos, Cie0, t);
        ckf.Hk(8:9,8) = -[fins.eth.cl; fins.eth.sl];
        zk = [fins.vn-gps(kgps,1:3)'; fins.pos-gps(kgps,4:6)'; qq2phi(fins.qnb,m2qua(Cns))];
        fkf = fkfupdate(ckf, fkf, zk);  % federated KF
    else
        ckf = kfupdate(ckf);
        fkf = fkfupdate(ckf, fkf);
    end
    [ckf, ins] = kffeedback(ckf, ins, 1, 'avp');
    [fkf{3}, fins] = kffeedback(fkf{3}, fins, 1, 'avp');
    % save results
    avpc(ki,:)  = [ins.avp', t];
    avpf(ki,:)  = [fins.avp', t];
    xkpkc(ki,:) = [ckf.xk; diag(ckf.Pxk); t]';
	xkpk1(ki,:) = [fkf{1}.xk; diag(fkf{1}.Pxk); t]';
	xkpk2(ki,:) = [fkf{2}.xk; diag(fkf{2}.Pxk); t]';
    xkpkf(ki,:) = [fkf{3}.xk; diag(fkf{3}.Pxk); t]';
    ki = timebar;
end
[avpc,avpf,xkpkc,xkpk1,xkpk2,xkpkf] = no0s(avpc,avpf,xkpkc,xkpk1,xkpk2,xkpkf);
insplot(avpc);
avpcmpplot(avpc, avpf);
kfplot(xkpkc);
kfplot(xkpkf);
% kfplot(xkpk1);
% kfplot(xkpk2);

