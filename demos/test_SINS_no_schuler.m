% SINS error with no Schuler cycle under special trajectory.
% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/01/2021
glvs
% trajectory simu
xxx = [];
seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'uniform',      84*60/2);
seg = trjsegment(seg, 'turnleft',     45, 2);
trj = trjsimu([[0;0;0]; [0;0;0]; glv.pos0], seg.wat, 1, 6);
insplot(trj.avp);
imuplot(trj.imu);
% inspure(trj.imu, trj.avp0, 'V');
% pure SINS
imuerr = imuerrset(0.00, [100;0;0]);
inspure(imuadderr(trj.imu,imuerr), trj.avp0, 'V');
