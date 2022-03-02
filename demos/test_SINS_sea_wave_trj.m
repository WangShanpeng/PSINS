% Attitude sway and constant velocity trajectory simulation.
% See also  imusway.
% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 10/01/2021
glvs
ts = 1/10;
avp0 = avpset([0;0;30], 10, glv.pos0);
seg = trjsegment([], 'init',         norm(avp0(4:6)));
seg = trjsegment(seg, 'uniform',     360);
trj = trjsimu(avp0, seg.wat, ts, 1); % insplot(trj.avp)
t = trj.avp(:,end);
ap = [[15*sin(2*pi/9*t), 25*sin(2*pi/11*t), avp0(3)/glv.deg+20*sin(2*pi/10*t)]*glv.deg, trj.avp(:,7:10)];
[imu, avp00, avp] = ap2imu(ap);  % insplot(avp)
avp1 = inspure(imu, avp00);  % insplot(avp1)
avpcmpplot(avp1, avp);
