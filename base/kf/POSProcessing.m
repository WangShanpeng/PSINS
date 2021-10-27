function [ps, psf] = POSProcessing(kf, ins, imu, vpGPS, fbstr, ifbstr)
% POS forward and backward data processing.
% States: phi(3), dvn(3), dpos(3), eb(3), db(3), lever(3), dT(1), 
%         dKg(9), dKa(6). (total states 6*3+1+9+6=34)
%
% Prototype: ps = POSProcessing(kf, ins, imu, posGPS, fbstr, ifbstr)
% Inputs: kf - Kalman filter structure array from 'kfinit'
%         ins - SINS structure array from 'insinit'
%         imu - SIMU data including [wm, vm, t]
%         vpGPS - GPS velocity & position [lat, lon, heigth, tag, tSecond]
%                 or [vE, vN, vU, lat, lon, heigth, tag, tSecond], where
%                 'tag' may be omitted.
%         fbstr,ifbstr - Kalman filter feedback string indicator for
%               forward and backward processing. if ifbstr=0 then stop
%               backward processing
% Output: ps - a structure array, its fields are
%             avp,xkpk  - forward processing avp, state estimation and
%                         variance
%             iavp,ixkpk  - backward processing avp, state estimation and
%                         variance
%
% Example:
%     psinstypedef(346);
%     davp0 = avperrset([30;-30;30], [0.01;0.01;0.03], [0.01;0.01;0.03]);
%     lever = [0.; 0; 0]; dT = 0.0; r0 = davp0(4:9)';
%     imuerr = imuerrset(0.01,100,0.001,1, 0,0,0,0, [0;0;1000],0,[0;0;0;0;10;10],0);
%     ins = insinit([att; pos], ts);
%     kf = kfinit(ins, davp0, imuerr, lever, dT, r0);
%     ps = POSProcessing(kf, ins, imu, vpGPS, 'avped', 'avp');
%     psf = POSFusion(ps.avp, ps.xkpk, ps.iavp, ps.ixkpk);
%     POSplot(psf);
% 
% See also  sinsgps, insupdate, kfupdate, POSFusion, posplot.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/01/2014, 08/02/2015
    if ~exist('fbstr','var'),  fbstr = 'avp';  end
    if ~exist('ifbstr','var'),  ifbstr = fbstr;  end
    len = length(imu); [lenGPS,col] = size(vpGPS);
    if col==4||col==7, vpGPS = [vpGPS(:,1:end-1),ones(lenGPS,1),vpGPS(:,end)]; end % add tag
    imugpssyn(imu(:,7), vpGPS(:,end));
    ts = ins.ts; nts = kf.nts; nn = round(nts/ts);
    dKga = zeros(15,1);
    %% forward navigation
	[avp, xkpk, zkrk] = prealloc(ceil(len/nn), kf.n+1, 2*kf.n+1, 2*kf.m+1); ki = 1;
    Qk = zeros(length(vpGPS), kf.n+1);  Rk = zeros(length(vpGPS), 8);  kki = 1;  zk = zeros(size(kf.Hk,1),1);
    timebar(nn, len, 'SINS/GPS POS forward processing.');
    for k=1:nn:(len-nn+1)
        k1 = k+nn-1; wvm = imu(k:k1,1:6); t = imu(k1,7);
        ins = insupdate(ins, wvm);  ins = inslever(ins);
        kf.Phikk_1 = kffk(ins);
        kf = kfupdate(kf);
        [kgps, dt] = imugpssyn(k, k1, 'F');
        if kgps>0 
            if vpGPS(kgps,end-1)>=1 % tag OK
                kf.Hk = kfhk(ins);
                if size(kf.Hk,1)==6
                    zk = [ins.vnL-ins.an*dt; ins.posL-ins.Mpvvn*dt]-vpGPS(kgps,1:6)';
                else
                    zk = ins.posL-ins.Mpvvn*dt-vpGPS(kgps,1:3)';
                end
            	kf = kfupdate(kf, zk, 'M');
            end
            Qk(kki,:) = [diag(kf.Qk); t]';
            Rk(kki,:) = [diag(kf.Rk); kf.beta; t]';  kki = kki+1;
        end
        [kf, ins] = kffeedback(kf, ins, nts, fbstr);
        dKg = ins.Kg-eye(3); dKa = ins.Ka-eye(3);
        dKga = [dKg(:,1);dKg(:,2);dKg(:,3); dKa(:,1);dKa(2:3,2);dKa(3,3)];
        avp(ki,:) = [ins.avpL; ins.eb; ins.db; ins.lever; ins.tDelay; dKga; t]';
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';
        zkrk(ki,:) = [zk; diag(kf.Rk); t]'; ki = ki+1;
        timebar;
    end
    avp(ki:end,:)=[]; xkpk(ki:end,:)=[]; zkrk(ki:end,:)=[]; Qk(kki:end,:)=[];  Rk(kki:end,:)=[];
    ps.avp = avp; ps.xkpk = xkpk; ps.zkrk = zkrk; ps.Qk = [sqrt(Qk(:,1:end-1)),Qk(:,end)];  ps.Rk = [sqrt(Rk(:,1:6)),Rk(:,7:8)];
    if ~ischar(ifbstr), return; end
    %% reverse navigation
    [ikf, iins, idx] = POSReverse(kf, ins);
    [iavp, ixkpk, izkrk] = prealloc(ceil(len/nn), kf.n+1, 2*kf.n+1, 2*kf.m+1); ki = 1;
    timebar(nn, len, 'SINS/GPS POS reverse processing.');
    for k=k1:-nn:(1+nn)
        ik1 = k-nn+1; wvm = imu(k:-1:ik1,1:6); wvm(:,1:3) = -wvm(:,1:3); t = imu(ik1-1,7);
        iins = insupdate(iins, wvm);  iins = inslever(iins);
        ikf.Phikk_1 = kffk(iins);
        ikf = kfupdate(ikf);
        [kgps, dt] = imugpssyn(ik1, k, 'B');
        if kgps>0 
            if vpGPS(kgps,end-1)>=1 % tag OK
                ikf.Hk = kfhk(iins);
                if size(ikf.Hk,1)==6
                    zk = [iins.vnL-iins.an*dt; iins.posL-iins.Mpvvn*dt]-[-vpGPS(kgps,1:3),vpGPS(kgps,4:6)]';
                else
                    zk = iins.posL-iins.Mpvvn*dt-vpGPS(kgps,1:3)';
                end
            	ikf = kfupdate(ikf, zk, 'M');
            end
        end
        [ikf, iins] = kffeedback(ikf, iins, nts, ifbstr);
        dKg = iins.Kg-eye(3); dKa = iins.Ka-eye(3);
        dKga = [dKg(:,1);dKg(:,2);dKg(:,3); dKa(:,1);dKa(2:3,2);dKa(3,3)];
        iavp(ki,:) = [iins.avpL; iins.eb; iins.db; iins.lever; iins.tDelay; dKga; t]';
        ixkpk(ki,:) = [ikf.xk; diag(ikf.Pxk); t]';
        izkrk(ki,:) = [zk; diag(ikf.Rk); t]';     ki = ki+1;
        timebar;
    end
    iavp(ki:end,:)=[]; ixkpk(ki:end,:)=[]; izkrk(ki:end,:)=[];  
    iavp = flipud(iavp); ixkpk = flipud(ixkpk); izkrk = flipud(izkrk); % reverse inverse sequence
    iavp(:,idx) = -iavp(:,idx);  ixkpk(:,idx) = -ixkpk(:,idx);
    ps.iavp = iavp; ps.ixkpk = ixkpk; ps.izkrk = izkrk;
    if nargout==2
        psf = POSFusion(ps.avp, ps.xkpk, ps.iavp, ps.ixkpk);
    end
    
function [ikf, iins, idx] = POSReverse(kf, ins)
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/01/2014
    iins = ins;
    iins.eth.wie = -iins.eth.wie;
    iins.vn = -iins.vn; iins.eb = -iins.eb; iins.tDelay = -iins.tDelay;
    ikf = kf;
    Pd=diag(ikf.Pxk); ikf.Pxk = diag([10*Pd(1:19);Pd(20:end)]);
    idx = [4:6,10:12,19];   % vn,eb,dT  (dKg no reverse!)
    ikf.xk(idx) = -ikf.xk(idx); % !!!
    