% GPS single-point positioning simulation.
% See also  test_GPS_DD, test_GPS_SINS_tightly_coupled.
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/08/2013
ggpsvars
len = 2000;
[eph, obs, ot] = rnx210('abpo0080.15n', len);
[eph, obs, ot] = rnx210('01.15n', len);
tp = unique(obs(:,ot.tp));
len = length(tp);
recPos = zeros(4,1); pvt = zeros(len,17); mygps = zeros(len,7);
timebar(1, len, 'GPS single-point positioning.');
for k=1:len
    tpi = tp(k);
    idx = obs(:,ot.tp)==tpi;
    obsi = obs(idx,:); ephi = eph(obsi(:,ot.ei),:);
    [satpv, clkerr] = satPosVelBatch(tpi-obsi(:,ot.C1)/ggps.c, ephi);
%     [pvti, vp, res] = lspvt(recPos, satpv, [obsi(:,ot.C1)+clkerr(:,2)*ggps.c]);
    [pvti, vp, res] = lspvt(recPos, satpv, [obsi(:,ot.C1)+clkerr(:,2)*ggps.c,obsi(:,ot.D1)*ggps.lambda1]);
    pvt(k,:) = [pvti;tpi]';  mygps(k,:) = [vp;tpi]';   recPos = pvti(1:4);
    timebar;
end
pvtplot(pvt, 't0h');
% gps = load('01dgps.dat'); gps=[gps(:,7:9), gps(:,3:5)*glv.deg, gps(:,2)];
% avpcmpplot(mygps, gps, 'vp');
