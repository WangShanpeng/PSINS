function mu = msmu(simu, mavp, t0, t1)
% Constant installation error angles 'mu' estimation between master- & slave- INS
% using transfer align method.
%
% Prototype: avp = insinstant(imu, avp, t0, t1)
% Inputs: simu - slave IMU
%         mavp - master AVP
%         t0 - start time in second.
%         t1 - end time in second.
% Output: mu - installation error angles
%
% Example:
%    glvs;
%    load([glv.datapath,'transfer_align_fog_mems.mat']); % imuplot(mimu); insplot(fogavp);
%    mu = msmu(mimu, fogavp, 2, 98);
%
% See also  aa2mu, aaddmu, test_align_transfer.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/01/2021
global glv
    if nargin<4, t1 = min(simu(end,end),mavp(end,end)); end
    if nargin<3, t0 = max(simu(1,end),mavp(1,end)); end
    simu = datacut(simu(:,[1:6,end]), t0, t1);   mavp = datacut(mavp(:,[1:9,end]), t0, t1);
    imuavp = combinedata(simu, mavp(:,[1:9,end]));
    ts = diff(simu(1:2,end));
    ebdb=zeros(6,1);
    qnbs = a2qua(mavp(1,1:3)'); vns = mavp(1,4:6)'; % slave-INS init
    eth = earth(mavp(1,7:9)', mavp(1,4:6)');
    kf = msmukfinit(ts);
    len = length(imuavp); [res, xkpk] = prealloc(len, 7, 2*kf.n+1);  kk=1;
    timebar(1, len, 'Installation error angles estimation between master- & slave- INS.');
    for k=1:len
        [phim, dvbm] = cnscl(imuavp(k,1:6)-ebdb'*ts); t = imuavp(k,7);
        Cnbs = q2mat(qnbs);
        dvn = Cnbs*dvbm; vns = vns + dvn + eth.gcc*ts;  % slave-INS velocity updating
        qnbs = qupdt(qnbs, phim-Cnbs'*eth.wnin*ts); 	% slave-INS attitude updating
        kf.Phikk_1(1:6,1:3) = [-askew(eth.wnin*ts)+glv.I33; askew(dvn)];
            kf.Phikk_1(1:3,7:9) = -Cnbs*ts; kf.Phikk_1(4:6,10:12) = Cnbs*ts;
        kf = kfupdate(kf);
        mavpk = imuavp(k,8:16)';
        if mavpk(7)>0.1 && norm(phim)/ts<10*glv.dps  % master-INS is valid
            qnbm = a2qua(mavpk(1:3));  vnm = mavpk(4:6);  posm = mavpk(7:9);
            eth = earth(posm, vnm);
            kf.Hk(1:3,13:15) = -Cnbs;
            kf = kfupdate(kf, [qq2phi(qnbs,qnbm); vns-vnm], 'M');
            qnbs = qdelphi(qnbs, kf.xk(1:3));  kf.xk(1:3)=0;  % feedback
            ebdb = ebdb + kf.xk(7:12);  kf.xk(7:12)=0;
            res(kk,:) = [qq2phi(qmul(qnbs,rv2q(-kf.xk(13:15))),qnbm); vns-vnm; t]; % record
            xk = kf.xk;  xk(7:12) = ebdb;
            xkpk(kk,:) = [xk; diag(kf.Pxk); t]';  kk=kk+1;
        end
        timebar;
    end
    res(kk:end,:)=[];  xkpk(kk:end,:)=[];
    msmuplot(res, xkpk);
    mu = xkpk(end,13:15)';

function kf = msmukfinit(ts)
global glv;
    % states: phi, dvn, eb, db, mu
    kf.Phikk_1 = eye(15);
    kf.Hk = [eye(6), zeros(6,9)];  % Z = [phi-Cnb*mu; dvn];
    kf.Qt = diag([[1;1;1]*glv.dpsh; [1;1;1]*glv.ugpsHz; zeros(9,1)])^2;
    kf.Rk = diag([[1;1;1]*10*glv.min; [.1;.1;.1]])^2;
    kf.Pxk = diag([[10;10;10]*glv.deg; [10;10;10]; [1000;1000;1000]*glv.dph; [10;10;10]*glv.mg; ...
        [1;1;1]*glv.deg])^2;
    kf = kfinit0(kf, ts);

function msmuplot(res, xkpk)
global glv;
    t = xkpk(:,end); len = length(t);
    myfigure,
    subplot(321),plot(t, xkpk(:,1:3)/glv.min), xygo('phi'), plot(t,res(:,1:3)/glv.min,'-.'),
    subplot(322),plot(t, xkpk(:,4:6)), xygo('dV');          plot(t,res(:,4:6),'-.'),
    subplot(323),plot(t, xkpk(:,7:9)/glv.dph), xygo('eb'),
    subplot(324),plot(t, xkpk(:,10:12)/glv.ug), xygo('db'),
    subplot(325),plot(t, xkpk(:,13:15)/glv.min), xygo('mu'),plot(t,repmat(xkpk(end,13:15),len,1)/glv.min,'-.'),
    title(sprintf('\\mu_{end} = %.2f, %.2f, %.2f (\\prime)', xkpk(end,13)/glv.min, xkpk(end,14)/glv.min, xkpk(end,15)/glv.min));
    myfigure,
    xkpk(:,15+[1:15]) = sqrt(xkpk(:,15+[1:15]));
    subplot(321),plot(t, xkpk(:,15+[1:3])/glv.min), xygo('phi');
    subplot(322),plot(t, xkpk(:,15+[4:6])), xygo('dV');
    subplot(323),plot(t, xkpk(:,15+[7:9])/glv.dph), xygo('eb');
    subplot(324),plot(t, xkpk(:,15+[10:12])/glv.ug), xygo('db');
    subplot(325),plot(t, xkpk(:,15+[13:15])/glv.min), xygo('mu');

