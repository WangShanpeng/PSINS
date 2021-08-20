function igsplot(imu, gps, s)
% IMU & GPS synchronization analysis plot.
%
% Prototype: igsplot(imu, gps, s)
% Inputs: imu - IMU data array.
%         gps - GPS dada array.
%         s - acc scale.
%          
% See also  imuplot, gpsplot, imugpssyn.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 24/05/2021
    if nargin<3, s=1; end
    myfig;
    ts = diff(imu(1:2,end));
    subplot(211), plot(imu(:,end), imu(:,4:5)/ts*s, gps(1:end-1,end), diff(gps(:,1:2))); xygo('f_{x,y}, V_{E,N}');
    subplot(212), plot(imu(:,end), (imu(:,6)/ts-9.8)*s, gps(:,end), gps(:,3)); xygo('f_z, V_U');