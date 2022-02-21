% Federated Kalman filter transfer align simulation. states include:
% [phi, eb, dvn, db, mu]'.
% Please run 'test_align_transfer_trj.m' &
%            'test_align_transfer_imu_simu.m' beforehand!!!
% See also  test_align_transfer_trj, test_align_transfer_imu_simu.
%           test_align_transfer.
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/01/2022
glvs
psinstypedef('test_align_transfer_fkf_def');
load([glv.datapath,'trj_transfer.mat']);
load([glv.datapath,'imu_transfer.mat']);  % imuplot(trj.imu); insplot(trj.avp);
ts = trj.ts;
qnbs = a2qua(trj.avp0(1:3)); vns = trj.avp0(4:6); % slave INS init
ckf = kfinit(ts, imuerr);
fkf = fkfinit(ckf, {[1:6,13:15],[1:12]}, {1:3, 4:6}, [50/100; 50/100], 0, 3);
len = 30/ts; length(trj.imu);
[res, xkpk, xkpk1, xkpk2, xkpkf] = prealloc(len, 7, 2*ckf.n+1, 2*fkf{1}.n,2*fkf{2}.n,2*fkf{3}.n);
timebar(1, len, 'Federated KF transfer alignment simulaton.');
for k=1:len
    [phim, dvbm] = cnscl(imu(k,:)); t = imu(k,7);
    qnbm = a2qua(trj.avp(k,1:3)');  vnm = trj.avp(k,4:6)'; posm = trj.avp(k,7:9)'; % master INS
    eth = earth(posm, vnm);
    Cnbs = q2mat(qnbs);
    dvn = Cnbs*dvbm; vns = vns + dvn + eth.gcc*ts;    % slave INS velocity
    qnbs = qupdt(qnbs, phim-Cnbs'*eth.wnin*ts);  % slave INS attitude
    ckf.Phikk_1([1:3,7:9],1:3) = [-askew(eth.wnin*ts)+glv.I33; askew(dvn)];
        ckf.Phikk_1(1:3,4:6) = -Cnbs*ts; ckf.Phikk_1(7:9,10:12) = Cnbs*ts;
    ckf.Hk(1:3,13:15) = -Cnbs;
    ckf = kfupdate(ckf, [qq2phi(qnbs,qnbm); vns-vnm]);
    [fkf, testHk] = fkfupdate(ckf, fkf, [qq2phi(qnbs,qnbm); vns-vnm]);
    res(k,:) = [qq2phi(qnbs,qnbsk(k,:)'); vns-vnm; t]'; 
    xkpk(k,:)  = [ckf.xk; diag(ckf.Pxk); t]';
    xkpk1(k,:) = [fkf{1}.xk; diag(fkf{1}.Pxk)]';
    xkpk2(k,:) = [fkf{2}.xk; diag(fkf{2}.Pxk)]';
    xkpkf(k,:) = [fkf{3}.xk; diag(fkf{3}.Pxk)]';
    timebar;
end
kfplot(xkpk, res, imuerr, mub, xkpk1, xkpk2, xkpkf);
return;
t = xkpk(:,end); len = length(t);
myfig,
subplot(421),plot(t, xkpk(:,1:3)/glv.min), xygo('phi'),  plot(t,res(:,1:3)/glv.min,'-.'),  plot(t, xkpkf(:,1:3)/glv.min, '-.', 'linewidth',2); ylim([-10,40]); title('( a )');
subplot(423),plot(t, xkpk(:,4:6)/glv.dph), xygo('eb'),   plot(t, xkpkf(:,4:6)/glv.dph, '-.', 'linewidth',2), ylim([-20,20]); title('( c )');
subplot(425),plot(t, xkpk(:,10:12)/glv.ug), xygo('db'),  plot(t, xkpk2(:,[10:12])/glv.ug, '-.', 'linewidth',2); title('( e )');
subplot(427),plot(t, xkpk(:,13:15)/glv.min), xygo('mu'), plot(t, xkpk1(:,[7:9])/glv.min, '-.', 'linewidth',2); ylim([-10,40]); title('( g )');
sxkpk(:,15+[1:15]) = sqrt(xkpk(:,15+[1:15]));  sxkpkf(:,6+[1:6]) = sqrt(xkpkf(:,6+[1:6]));
sxkpk1(:,9+[1:9]) = sqrt(xkpk1(:,9+[1:9]));  sxkpk2(:,12+[1:12]) = sqrt(xkpk2(:,12+[1:12]));
subplot(422),semilogy(t, sxkpk(:,15+[1:3])/glv.min), xygo('( P_\phi )^{1/2} / ( \prime )'); semilogy(t, sxkpkf(:,6+[1:3])/glv.min, '-.', 'linewidth',2); ylim([0.01,1000]); title('( b )');
subplot(424),semilogy(t, sxkpk(:,15+[4:6])/glv.dph), xygo('( P_\epsilon )^{1/2} / ( (\circ)/h )'); semilogy(t, sxkpkf(:,6+[4:6])/glv.dph, '-.', 'linewidth',2); ylim([1,10]); title('( d )');
subplot(426),semilogy(t, sxkpk(:,15+[10:12])/glv.ug), xygo('( P_\nabla )^{1/2} / \mug');  semilogy(t, sxkpk2(:,12+[10:12])/glv.ug, '-.', 'linewidth',2); ylim([1e1,1e4]); title('( f )');
subplot(428),semilogy(t, sxkpk(:,15+[13:15])/glv.min), xygo('( P_\mu )^{1/2} / ( \prime )');  semilogy(t, sxkpk1(:,9+[7:9])/glv.min, '-.', 'linewidth',2); ylim([1e-2,1e3]); title('( g )');

myfig,
subplot(221),plot(t, xkpk(:,1:3)/glv.min), xygo('phi'),  plot(t, xkpkf(:,1:3)/glv.min, '-.', 'linewidth',2),  plot(t,res(:,1:3)/glv.min,'-.'); ylim([-10,40]);
legend('CKF \phi_E', 'CKF \phi_N', 'CKF \phi_U', 'FKF \phi_E', 'FKF \phi_N', 'FKF \phi_U', '\phi_E true', '\phi_N  true', '\phi_U  true')
