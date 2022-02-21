% SINS/GPS/CNS centralized v.s. federated KF with 21-state include:
%       phi(3), dvn(3), dpos(3), eb(3), db(3), lv(3), mu(3)
% Please run 'test_SINS_trj.m' to generate 'trj10ms.mat' beforehand!!!
% See also  sinsgps, test_SINS_GPS_186, test_SINS_CNS_184.
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/01/2022
glvs
trj = trjfile('trj10ms.mat');
[nn, ts, nts] = nnts(2, trj.ts);
% GPS simulator
lever = [1; 2; 3]; rk1 = vperrset(0.1, 1);
davp0 = avperrset([0.5;-0.5;20], 0.01, [1;1;3]);
gps = gpssimu(trj.avp, davp0(4:6), davp0(7:9), 1, lever, 0.0);  % gpsplot(gps)
% CNS simulator
mu = [1;2;5]*glv.min;  rk2 = [[10;10;30]*glv.sec];
% [qis, utc0] = cnssimu(trj.avp, rk2, mu, [2021;11;22;12*3600; -0.1;37]);
% SINS init
imuerr = imuerrset(0.03, [100;100;100], 0.001, 5);
imu = imuadderr(trj.imu, imuerr);  % imuplot(imu)
ins = insinit(trj.avp0(1:9), ts, davp0);  len = length(imu);
Cie0 = cnsCie(utc0(1:3), utc0(4),  utc0(5), utc0(6));
% centralized KF
psinstypedef('test_SINS_GPS_CNS_def');
ckf = kfinit(ins, davp0, imuerr, [rk1;rk2], lever, mu);
ckf = kfinit0(ckf, nts);
% federated KF setting
fins = ins;
fkf = fkfinit(ckf, {1:18,[1:15,19:21]}, {1:6,7:9}, [1/2; 1/3], 0);
% alloc memory
[avpc, xkpkc] = prealloc(fix(len/nn), 10, 2*ckf.n+1);
[avp1, xkpk1] = prealloc(fix(len/nn), 10, 2*fkf{1}.n+1);
[avp2, xkpk2] = prealloc(fix(len/nn), 10, 2*fkf{2}.n+1);
[avpf, xkpkf] = prealloc(fix(len/nn), 10, 2*fkf{3}.n+1);
ki = timebar(nn, len, '21-state SINS/GPS/CNS centralized v.s. federated KF simulation.');
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
        ins = inslever(ins);
        ckf.Hk(1:6,16:18) = [-ins.CW;-ins.MpvCnb];
        ckf.Hk(8:9,8) = -[ins.eth.cl; ins.eth.sl];  ckf.Hk(7:9,19:21) = ins.Cnb;
        zk = [ins.vnL-gps(kgps,1:3)'; ins.posL-gps(kgps,4:6)'; qq2phi(ins.qnb,m2qua(Cns))];
        ckf = kfupdate(ckf, zk);  % centralized KF
        Cns = cnsCns(qis(k1,1:3)', fins.pos, Cie0, t);
        fins = inslever(fins);
        ckf.Hk(1:6,16:18) = [-fins.CW;-fins.MpvCnb];
        ckf.Hk(8:9,8) = -[fins.eth.cl; fins.eth.sl];  ckf.Hk(7:9,19:21) = fins.Cnb;
        zk = [fins.vnL-gps(kgps,1:3)'; fins.posL-gps(kgps,4:6)'; qq2phi(fins.qnb,m2qua(Cns))];
        fkf = fkfupdate(ckf, fkf, zk);  % federated KF
    else
        ckf = kfupdate(ckf);
        fkf = fkfupdate(ckf, fkf);
    end
    [ckf, ins] = kffeedback(ckf, ins, 1, 'avp');
    [fkf{3}, fins, xfb] = kffeedback(fkf{3}, fins, 1, 'avp');   if fkf{3}.fkfreset==0, for kk=1:3, fkf{kk}.xk(1:15)=fkf{kk}.xk(1:15)-xfb; end; end
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
% insplot(avpc);
% avpcmpplot(avpc, avpf);
inserrplot(xkpkc(:,[1:18,end]));  subplot(428), plot(xkpkc(:,end), xkpkc(:,19:21)/glv.min); xygo('mu');
inserrplot(xkpk1(:,[1:18,end]));
inserrplot(xkpk2(:,[1:18,end]));  delete(subplot(427)), subplot(428), hold off; plot(xkpk2(:,end), xkpk2(:,16:18)/glv.min); xygo('mu');
inserrplot(xkpkf(:,[1:18,end]));  delete(subplot(427))
return;
insserrplot(xkpkc(:,[22:39,end])); subplot(428), plot(xkpkc(:,end), sqrt(xkpkc(:,40:42))/glv.min); xygo('mu');
insserrplot(xkpk1(:,[19:36,end]));
insserrplot(xkpk2(:,[19:36,end])); delete(subplot(427)), subplot(428), plot(xkpk2(:,end), sqrt(xkpk2(:,34:36))/glv.min); xygo('mu');


