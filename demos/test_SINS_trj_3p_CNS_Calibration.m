% 3-position trajectory simulation for SINS/CNS installation angles calibration,
% first rotated by yaw then pitch.
% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 20/09/2021
glvs
ts = 1/100;
att0 = [0.1;0;10]*glv.deg;
pos0 = posset(34, 116, 480);
att = attrottt(att0, [1, 0,0,1, 180*glv.deg, 20,50,50; ...
                      2, 1,0,0, 30*glv.deg, 10, 10, 60], 0.01);
len = length(att);
avp = [att(:,1:3),zeros(len,3),repmat(pos0',len,1),att(:,4)-att(1,4)];
imu = avp2imu(avp);
imuplot(imu);
trj.imu = imu; trj.avp = avp; trj.avp0 = avp(1,1:9)'; trj.ts = ts;
trjfile('trj10ms.mat', trj);


