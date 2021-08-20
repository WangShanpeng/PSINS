function [static, moving] = imustmv(imu, angrate)
% IMU static or moving test.
%
% Prototype: stmv = imustmv(imu, angrate)
% Inputs: imu - imu data array
%         angrate - angluar rate threshold
% Outputs: static - static flag
%          moving - moving flag
%
% See also  zuptest, imustatic, imudot.

% Copyright(c) 2009-2016, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 24/10/2016
global glv
    ts = diff(imu(1:2,end));
    if nargin<2, angrate = 200*glv.dph; end
    rate = normv(imu(:,1:3));
    b = ones(200); b = b/sum(b);
    rate = filtfilt(b, 1, rate);
    stmv = rate/ts>angrate;
    stmv = diff(stmv);
    static = find(stmv==-1);  static = [1; static];
    moving = find(stmv==1);
    figure, subplot(211), plot(imu(:,end), imu(:,1:3)/ts/glv.dph); xygo('Gyro');
        plot(imu(static,end), static*0, 'om',imu(moving,end), moving*0, 'ob');
    subplot(212), plot(imu(:,end), imu(:,4:6)/ts); xygo('Acc');
        plot(imu(static,end), static*0, 'om',imu(moving,end), moving*0, 'ob');        