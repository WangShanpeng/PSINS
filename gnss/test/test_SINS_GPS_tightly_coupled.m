% SINS/GPS tightly-coupled integrated navigation processing.
% See also  test_GPS_PVT, test_SINS_GPS_186.
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/04/2022
ggpsvars
psinstypedef('test_SINS_GPS_tightly_def');
load insgpseo_tc.mat;  % vars: imu ephs obss t0
[nn, ts, nts] = nnts(1, diff(imu(1:2,end)));  % imuplot(imu);  
vp = getgnssvp(ephs, obss, 185500, 1);
att = aligni0(datacut(imu,0,50), vp(4:6)); 
avp0 = [att; vp]; % avp=inspure(imu, [att;vp]); 
davp = avperrset([10;10;60], 1, 100);
imuerr = imuerrset(0.05,1000,0.001,10);
ins = insinit(avp0, ts);
kf = kfinit(ins, davp, imuerr);
len = fix(134/ts);   [avp, xkpk] = prealloc(len/nn, 10, 2*kf.n+1);  kk=1;
recPos = zeros(4,1); pvt = zeros(len,17); mygps = zeros(len,7);
timebar(nn, len, 'SINS/GPS tightly-coupled processing'); ki = 1;
for k=1:nn:len-nn*2
    k1 = k+nn-1;
    wvm = imu(k:k1,1:6);  tp = t0+imu(k1,7);
    ins = insupdate(ins, wvm);
    kf.Phikk_1 = kffk(ins);
    kf = kfupdate(kf);
    if mod(tp,1)<ts
        obsi = findgpsobs(tp);
        if size(obsi,1)<4, continue; end
        ephi = ephs(obsi(:,2),:);  CA = obsi(:,3);
        [satpv, clkerr] = satPosVelBatch(tp-CA/ggps.c, ephi);      % SPP
        [pvti, vp, res] = lspvt(recPos, satpv, CA+clkerr(:,2)*ggps.c); recPos = pvti(1:4);
        pvt(kk,:) = [pvti;tp]';  mygps(kk,:) = [vp;tp]'; kk=kk+1;
        ins = inslever(ins, 0*kf.xk(16:18));
        if 0 %% 0 for loosely-coupled, 1 for tightly-coupled
            kf.Hk = [zeros(3,6), eye(3), zeros(3,6), ins.MpvCnb, -ins.Mpvvn, zeros(3,2)];   
            kf = kfupdate(kf, ins.pos-mygps(kk-1,4:6)');
        else
            [satPos, clkCorr] = satPosBatch(tp-CA/ggps.c, ephi);
            [posxyz, Cen] = blh2xyz(ins.pos);  % not posL !
            [rho, LOS, AzEl] = rhoSatRec(satPos, posxyz, CA);
            el = AzEl(:,2); el(el<15*pi/180) = 1*pi/180;  P = diag(sin(el.^2));  % P = eye(size(P));
            delta_rho = CA + clkCorr(:,1)*ggps.c - rho;
            kf.Hk = kfhk(ins, LOS);
            kf.Rk = P^-1*10^2;
            kf = kfupdate(kf, delta_rho);
        end
    end
	[kf, ins] = kffeedback(kf, ins, nts, 'avp');
    avp(ki,:) = [ins.avp; tp]';
	xkpk(ki,:) = [kf.xk; diag(kf.Pxk); tp]';
	ki = ki+1;
    timebar;
end
avp(ki:end,:) = []; xkpk(ki:end,:) = [];  mygps(kk:end,:) = [];
kfplot(xkpk, avp, mygps); % avpcmpplot(avp(:,4:end),mygps,'vp');

