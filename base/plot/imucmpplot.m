function err = imucmpplot(imu0, imu1)
% IMUs comparison & errors plot.
%
% Prototype: err = imucmpplot(imu0, imu1)
% Inputs: imu0, imu1 - input IMU data
% Output: err - err = imu1 - imu0
%          
% See also  avpcmpplot, imuplot.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/10/2022
global glv
    ts1 = diff(imu1(1:2,end));
    imu0 = imuresample(imu0, ts1, imu0(1,end));
    t1 = max(imu0(1,end),imu1(1,end));
    t2 = min(imu0(end,end),imu1(end,end));
    imu0 = datacut(imu0, t1, t2);
    imu1 = datacut(imu1, t1, t2);
    len = min(length(imu0),length(imu1));
    err = [imu1(1:len,1:6)-imu0(1:len,1:6), imu1(1:len,end)];
    imuplot(err, glv.dph);