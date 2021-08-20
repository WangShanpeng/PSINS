% GPS/SINS tightly-coupled integrated navigation simulation
% See also  test_GPS_PVT, test_SINS_GPS_186.
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/10/2011, 17/07/2015
ggpsvars
psinstypedef('test_GPS_INS_static_tight_def');
[nn, ts, nts] = nnts(2, 1);
xyz0 = [4097216.6805,  4429119.0287, -2065771.3676]';  pos0 = xyz2blh(xyz0);
% [eph, obs, ot] = rnx210('abpo0080.15n', 2000);
avp0 = [[1;1;30]*glv.deg;0;0;0;pos0];
davp = [d2r([1;1;1]); [1;1;1]; poserrset(100)];
imuerr = imuerrset(0.01,100,0.001,10, 0.01,300,100,300);  % tactical gride SIMU
[imu, eth] = imustatic(avp0, ts, 3600, imuerr);  imu(:,end)=imu(:,end)-imu(1,end)+obs(1,ot.tp)-29;
ins = insinit(avp0, ts, davp);  ins.tauG=imuerr.taug; ins.tauA=imuerr.taua; 
kf = kfinit(ins, davp, imuerr);
len = 3600/ts;   [avp, xkpk] = prealloc(len/nn, 10, 2*kf.n+1);  kk=1;
recPos = zeros(4,1); pvt = zeros(len,17); mygps = zeros(len,7);
timebar(nn, len, 'SINS/GPS tightly-coupled simulation.'); ki = 1;
for k=1:nn:len-nn*2
    k1 = k+nn-1;
    wvm = imu(k:k1,1:6);  t = imu(k1,7);
    ins = insupdate(ins, wvm);
    kf.Phikk_1 = kffk(ins);
    kf = kfupdate(kf);
    if mod(k1,30)==0
        tpi = obs(1,ot.tp)+k1;
        idx = obs(:,ot.tp)==tpi;
        obsi = obs(idx,:);   ephi = eph(obs(idx,ot.ei),:);
        [satpv, clkerr] = satPosVelBatch(tpi-obsi(:,ot.C1)/ggps.c, ephi);      % SPP
        [pvti, vp, res] = lspvt(recPos, satpv, obsi(:,ot.C1)+clkerr(:,2)*ggps.c); recPos = pvti(1:4);
        pvt(kk,:) = [pvti;tpi]';  mygps(kk,:) = [vp;tpi]'; kk=kk+1;
        ins = inslever(ins, 0*kf.xk(16:18));
        if 1 %% 0 for loosely-coupled, 1 for tightly-coupled
            kf.Hk = [zeros(3,6), eye(3), zeros(3,6), ins.MpvCnb, -ins.Mpvvn, zeros(3,2)];   
            kf = kfupdate(kf, ins.pos-mygps(kk-1,4:6)');
        else
            C1 = obsi(:,ot.C1);
            [satPos, clkCorr] = satPosBatch(tpi-C1/ggps.c, ephi);
            [posxyz, Cne] = blh2xyz(ins.pos);  % not posL !
            [rho, LOS, AzEl] = rhoSatRec(satPos, posxyz, C1);
            el = AzEl(:,2); el(el<15*pi/180) = 1*pi/180;  P = diag(sin(el.^2));  % P = eye(size(P));
            delta_rho = C1 + clkCorr(:,1)*ggps.c - rho;
            kf.Hk = kfhk(ins, LOS);
            kf.Rk = P^-1*10^2;
            kf = kfupdate(kf, delta_rho);
        end
    end
	[kf, ins] = kffeedback(kf, ins, nts, 'avp');
    avp(ki,:) = [ins.avp; t]';
	xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';
	ki = ki+1;
    timebar;
end
avp(ki:end,:) = []; xkpk(ki:end,:) = [];  mygps(kk:end,:) = [];
kfplot(xkpk, avp, mygps);
return
