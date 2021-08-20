% Generate data for VC60 SINS/GNSS simulation. Please run 
% 'test_SINS_trj.m' to generate 'trj10ms.mat' beforehand!!!
% See also  test_SINS_trj, test_SINS_GPS_153, test_SINS_static.
% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 10/05/2021
glvs
trj = trjfile('trj10ms.mat');
% error setting
imuerr = imuerrset(0.01, 100, 0.001, 10);
imu = imuadderr(trj.imu, imuerr);
davp0 = avperrset([0.5;0.5;5], 0.1, [10;10;10]);
avp00 = avpadderr(trj.avp0, davp0);
% pure inertial navigation & error plot
gps = gpssimu(trj.avp, [0.02;0.02;0.05], poserrset(1,1,3), 0.1, [1;2;3], 0.01, 1);
imugps = combinedata(imu, gps);
imugps = imugps(:,[1:6,8:13,7]);
binfile([glv.rootpath,'\vc60\data\imugps.bin'], imugps);

