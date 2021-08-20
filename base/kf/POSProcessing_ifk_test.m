function POSProcessing_ifk_test()
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/12/2019
global glv
    ts = 1; nts = 1; nn = 1; T = 5000; len = T/ts;
    psinstypedef(186);
    %% forward navigation
    ts = 0.01;
    avp0 = avpset([1;10;30]*glv.deg, [100;200;10], [34;106;100]);
    davp0 = avperrset([60;-60;300], [0.1;0.1;0.3], [1;1;3]);
    ins = insinit(avpadderr(avp0,[[30;-30;30]; [1;1;3]; [1;1;3]]), ts); ins.nts = nts;
    Ft = etm(ins);
    iins = ins;
    iins.eth.wie = -iins.eth.wie;
    iins.vn = -iins.vn;
    iins.eth = ethupdate(iins.eth, iins.pos, iins.vn);
    iFt = etm(iins);
    xk = [davp0; [1;1;1]*glv.dph; [100;100;100]*glv.ug];
    xk1 = (eye(15)+Ft*ts)*xk;
    ixk = xk1; ixk([4:6,10:12]) = -ixk([4:6,10:12]);
    ixk1 = (eye(15)+iFt*ts)*ixk;
    ixk2 = (eye(15)-Ft*ts)*ixk;
    [ixk1,ixk2,xk]
    
    avp0 = avpset([1;10;30]*glv.deg, [0;0;0], [34;106;100]);
    davp0 = avperrset([1;-1;30], [0.1;0.1;0.3], [0.01;0.01;0.03]);
    imuerr = imuerrset(0, 0, 0, 0);
    imu = imustatic(avp0, ts, len, imuerr);
    ins = insinit(avpadderr(avp0,[[30;-30;30]; [0.1;0.1;0.3]; [0.01;0.01;0.03]]), ts); ins.nts = nts;
    kf = kfinit(ins, davp0, imuerr, [0;0;0], 0, [0;0;0]);
    kf.xk(1:9) = davp0;
	err = prealloc(ceil(len/nn), 19); ki = 1;
    timebar(nn, len, 'SINS/GPS POS forward processing.');
    for k=1:nn:(len-nn+1)
        k1 = k+nn-1; wvm = imu(k:k1,1:6); t = imu(k1,7);
        ins = insupdate(ins, wvm);  ins.vn(3) = 0; kf.xk(6) = 0;
        kf.Phikk_1 = kffk(ins);
        kf = kfupdate(kf);
        err(ki,:) = [aa2phi(ins.att,avp0(1:3));(ins.avp(4:9)-avp0(4:9)); kf.xk(1:9); t];  ki = ki+1;
        timebar;
    end
    err(ki:end,:)=[];
    %% reverse navigation
    [ikf, iins, idx] = POSReverse(kf, ins);
	ierr = prealloc(ceil(len/nn), 19); ki = 1;
    timebar(nn, len, 'SINS/GPS POS reverse processing.');
    for k=k1:-nn:(1+nn)
        ik1 = k-nn+1; wvm = imu(k:-1:ik1,1:6); wvm(:,1:3) = -wvm(:,1:3); t = imu(ik1-1,7);
        iins = insupdate(iins, wvm);  iins.vn(3) = 0; ikf.xk(6) = 0;
        ikf.Phikk_1 = kffk(iins);
        ikf = kfupdate(ikf);
        ierr(ki,:) = [aa2phi(iins.att,avp0(1:3));(iins.avp(4:9)-avp0(4:9)); ikf.xk(1:9); t-0.5];  ki = ki+1;
        timebar;
    end
    ierr(ki:end,:)=[];
%     ierr = flipud(ierr);  
    ierr(:,[4:6]) = -ierr(:,[4:6]);  ierr(:,9+[4:6]) = -ierr(:,9+[4:6]);
    err = [err;ierr];
    errr = avpcmpplot(err(:,[1:9,end]), err(:,10:end));
%     avpcmpplot(ierr(:,[1:9,end]), ierr(:,10:end));
    
function [ikf, iins, idx] = POSReverse(kf, ins)
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/01/2014
    iins = ins;
    iins.eth.wie = -iins.eth.wie;
    iins.vn = -iins.vn; iins.eb = -iins.eb; iins.tDelay = -iins.tDelay;
    ikf = kf;
    ikf.Pxk = 10*diag(diag(ikf.Pxk));
    idx = [4:6,10:12];   % vn,eb,dT,dvn  (dKg no reverse!)
    ikf.xk(idx) = -ikf.xk(idx); % !!!
    