% GPS data load and obs view.
% Copyright(c) 2009-2015, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/08/2015
ggpsvars
len = 2000;
xyz0 = [4097216.6805,  4429119.0287, -2065771.3676]';
[eph, obs, ot] = rnx210('abpo0080.15n', len);
[satpv, clkerr] = satPosVelBatch(obs(:,ot.tp)-obs(:,ot.C1)/ggps.c, eph(obs(:,ot.ei),:));
AzEl = satPos2AzEl(satpv, xyz0);
satplot(obs(:,ot.PRN), AzEl);
obsplot(obs(:,ot.tp:ot.PRN), obs(:,ot.PRN))
obsplot(obs(:,ot.tp:ot.PRN), obs(:,ot.C1))