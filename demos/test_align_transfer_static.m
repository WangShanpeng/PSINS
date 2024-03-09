% Transfer align simulation in static base to demo gyro bias estimation.
% See also  test_align_transfer.
% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/05/2023
glvs, glvwie(0);
psinstypedef('test_align_transfer_def');
avp0 = [[0;0;0]; [0;0;0]; glv.pos0];  ts = 0.1;  beta=zeros(3,1); Q=zeros(3,1);
imuerr = imuerrset(100, 1000, 0.1, 10);
imu = imustatic(avp0, ts, 60, imuerr);
qnbs = qaddphi(a2qua(avp0(1:3)),[10;20;30]*glv.min); vns = avp0(4:6); % slave INS init
kf = kfinit(ts, imuerr, beta, Q);
kf.Pxk(16:end,16:end)=0;  % no flexure deformation
kf.Pxk(3,3)=(10*glv.deg)^2;  kf.Pxk(9,9)=(100*glv.dph)^2;  kf.Pxk(15,15)=(10*glv.deg)^2;  % reset phi_U/eb_Z/mu_Z variance
len = length(imu); [res, xkpk] = prealloc(len, 7, 2*kf.n+1);
timebar(1, len, 'Transfer alignment simulaton.');
for k=1:len
    [phim, dvbm] = cnscl(imu(k,:)); t = imu(k,7);
    qnbm = a2qua(avp0(1:3));  vnm = avp0(4:6); posm = avp0(7:9); % master INS
    eth = earth(posm, vnm);
    Cnbs = q2mat(qnbs);
    dvn = Cnbs*dvbm; vns = vns + dvn + eth.gcc*ts;    % slave INS velocity
    qnbs = qupdt(qnbs, phim-Cnbs'*eth.wnin*ts);  % slave INS attitude
    kf.Phikk_1(1:6,1:3) = [-askew(eth.wnin*ts)+glv.I33; askew(dvn)];
        kf.Phikk_1(1:3,7:9) = -Cnbs*ts; kf.Phikk_1(4:6,10:12) = Cnbs*ts;
    kf.Hk(1:3,13:18) = [-Cnbs,-Cnbs]; % kf.Hk(1:2,:) = 0; % kf.Hk(3,[13:14,16:17])=0;
    kf = kfupdate(kf, [qq2phi(qnbs,qnbm); vns-vnm]);
    res(k,:) = [qq2phi(qnbs,a2qua(avp0(1:3))); vns-vnm; t]'; 
    xkpk(k,:) = [kf.xk; diag(kf.Pxk); t]';
%     qnbs = qdelphi(qnbs,kf.xk(1:3)); kf.xk(1:3) = 0;
%     vns = vns-kf.xk(4:6); kf.xk(4:6) = 0;
    timebar;
end
mub=zeros(3,1);  thetak=[imu(:,1:3)*0, imu(:,end)];  omegak=thetak;
kfplot(xkpk, res, imuerr, mub, thetak, omegak);
subplot(426); hold off; plot(xkpk(:,end), sqrt(xkpk(:,21+[3,15]))/glv.min); xygo('\phi_U, \mu_Z / \prime');
figure(gcf-1);
subplot(426); hold off; plot(xkpk(:,end), xkpk(:,[3,15])/glv.min); xygo('\phi_U, \mu_Z / \prime');


