% SINS pure inertial navigation simulation. Please run 
% 'test_SINS_trj.m' to generate 'trj10ms.mat' beforehand!!!
% See also  test_SINS_trj, test_SINS_GPS_153, test_SINS_static.
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/06/2011
glvs
trj = trjfile('trj10ms.mat');
%% error setting
imuerr = imuerrset(0.01, 100, 0.001, 10);
imu = imuadderr(trj.imu, imuerr);
davp0 = avperrset([0.5;0.5;5], 0.1, [10;10;10]);
avp00 = avpadderr(trj.avp0, davp0); 
trj = bhsimu(trj, 1, 10, 3, trj.ts);
%% pure inertial navigation & error plot
avp = inspure(imu, avp00, trj.bh, 1);
% avp = inspure(imu, avp00, 'f', 1);
avperr = avpcmpplot(trj.avp, avp);

