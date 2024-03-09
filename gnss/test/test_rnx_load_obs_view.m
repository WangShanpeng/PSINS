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
%%

t = (0:600:12*3600)';  % satellite position in inertial frame with a day
myfig, 
wie = ggps.wie; ggps.wie = 0;
for k=1:max(eph(:,1))
    n = find(eph(:,1)==k, 1, 'first');  if isempty(n), continue; end
    [satpv, clkerr] = satPosVelBatch(t, eph(n,:));
    plot3(satpv(:,1), satpv(:,2), satpv(:,3), '-'); axis equal; hold on
end
ggps.wie = wie;
xlabel('X / m'); ylabel('Y / m'); zlabel('Z / m');


