% IMU/AVP/GNSS data extract&transform. Please run 
% 'test_SINS_trj.m' to generate 'trj10ms.mat' beforehand!!!
% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/04/2021
glvs
trj = trjfile('trj10ms.mat');
%% data fabrication 
imu1 = [[trj.imu(:,[2,1]),-trj.imu(:,3)]/trj.ts/glv.dps, [trj.imu(:,[5,4]),-trj.imu(:,6)]/trj.ts/glv.g0];  % FRD,deg/s,g
avp1 = [[trj.avp(:,1:2),yawcvt(trj.avp(:,3),'cc180c360')]/glv.deg, trj.avp(:,4:6), trj.avp(:,[8,7])/glv.deg, trj.avp(:,9)]; % deg,c360 
gps1 = [trj.avp(1:10:end,[5,4]), -trj.avp(1:10:end,6), trj.avp(1:10:end,[8,7])/glv.deg, trj.avp(1:10:end,9)]; % FRD,deg
gps1(:,1:3)=gps1(:,1:3)+randn(size(gps1(:,1:3)))*0.01;
dd = [imu1,avp1,trj.avp(:,4:9)*0,trj.avp(:,10)+100]; dd(1:10:end,end-6:end-1)=gps1;
binfile('imuavpgps.bin', dd);
%% IMU/AVP/GNSS data extract&transform
dd = binfile('imuavpgps.bin', 22);
open dd;
imu = imurfu(imuidx(dd, [1:6,22],glv.dps,glv.g0,trj.ts),'frd');
avp = avpidx(dd,[7:12,14,13,15,22],1,1);
gps = gpsidx(dd,[17,16,-18,20,19,21,22],1);
[imu,avp,gps] = tshift(imu,avp,gps,10);
imuplot(imu); % imuplot(trj.imu);
insplot(avp); % insplot(trj.avp);
gpsplot(gps);
open imu
open avp
open gps
