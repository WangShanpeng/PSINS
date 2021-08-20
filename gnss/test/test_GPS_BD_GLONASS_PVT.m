% GPS/BD/GLONASS single-point positioning simulation.
% See also  test_GPS_PVT, test_GPS_SINS_tightly_coupled.
% Copyright(c) 2009-2015, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/08/2015
ggnss;
% [eph, obs, ot] = rnx302('cubb1870.15n', 3000); % save cubb1870 eph obs ot;
% load cubb1870
tp = unique(obs(:,ot.tp));
len = length(tp);
recPos = [4097216.6805  4429119.0287 -2065771.3676 0]'; pvt = zeros(len,17); mygps = zeros(len,7);
bdRecPos = recPos; bdpvt = pvt; mybd = mygps;
gloRecPos = recPos; glopvt = pvt; myglo = mygps;
timebar(1, len, 'GPS/BD/GLONASS single-point positioning.');
for k=1:len
    tpi = tp(k);
    obsk = obs(obs(:,ot.tp)==tpi,:);
    %% GPS
    idx = (obsk(:,ot.PRN)>100 & obsk(:,ot.PRN)<200);
    obsi = obsk(idx,:); ephi = eph(obsi(:,ot.ei),:);
    [satpv, clkerr] = satPosVelBatch(tpi-obsi(:,ot.GC1C)/ggps.c, ephi);
    [pvti, vp] = lspvt(recPos, satpv, [obsi(:,ot.GC1C)+clkerr(:,2)*ggps.c,obsi(:,ot.GD1C)*ggps.lambda1]);
    pvt(k,:) = [pvti;tpi]';  mygps(k,:) = [vp;tpi]';   recPos = pvti(1:4);
    %% BD
    bdtpi = tpi-14;
    idx = (obsk(:,ot.PRN)>500 & obsk(:,ot.PRN)<600);
    obsi = obsk(idx,:); ephi = eph(obsi(:,ot.ei),:);
    [satpv, clkerr] = bdsatPosVelBatch(bdtpi-obsi(:,ot.CC2I)/ggps.c, ephi);
    [pvti, vp] = lspvt(bdRecPos, satpv, [obsi(:,ot.CC2I)+clkerr(:,2)*gbd.c,obsi(:,ot.CD2I)*gbd.lambda1]);
    bdpvt(k,:) = [pvti;tpi]';  mybd(k,:) = [vp;tpi]';   bdRecPos = pvti(1:4);
    %% GLONASS
    glotpi = tpi-17;
    idx = (obsk(:,ot.PRN)>200 & obsk(:,ot.PRN)<300);
    obsi = obsk(idx,:); ephi = eph(obsi(:,ot.ei),:);  lambda1 = gglo.lambda1(ephi(:,gglo.ephs.frqNO)+8);
    [satpv, clkerr] = glosatPosVel(glotpi-obsi(:,ot.RC1C)/ggps.c, ephi);
    [pvti, vp] = lspvt(gloRecPos, satpv, [obsi(:,ot.RC1C)+clkerr(:,1)*gglo.c,obsi(:,ot.RD1C).*lambda1]);
    glopvt(k,:) = [pvti;tpi]';  myglo(k,:) = [vp;tpi]';   gloRecPos = pvti(1:4);
    timebar;
end
% pvtplot(pvt, 't0h', recPos(1:3));
% pvtplot(bdpvt, 't0h', recPos(1:3));
pvtplot(glopvt, 't0h', recPos(1:3));

