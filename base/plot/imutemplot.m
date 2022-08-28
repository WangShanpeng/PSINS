function imutemplot(imu, n)
% SIMU data plot where temperature in x-axis.
%
% Prototype: imutemplot(imu)
% Inputs: imu - SIMU data, =[wm, vm, temp, t]
%         n - mean count
%          
% See also  imutplot, imuplot, imumeanplot.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 04/08/2021
    global glv
    if nargin>1
        imu = imuresample(imu, n*diff(imu(1:2,end)));
        imutemplot(imu);
        return;
    end
    myfig;
    ts = diff(imu(1:2,end));
    subplot(321), plot(imu(:,7), imu(:,1)/ts/glv.dph); xygo('Temp', 'wxdph');
    subplot(323), plot(imu(:,7), imu(:,2)/ts/glv.dph); xygo('Temp', 'wydph');
    subplot(325), plot(imu(:,7), imu(:,3)/ts/glv.dph); xygo('Temp', 'wzdph');
    subplot(322), plot(imu(:,7), imu(:,4)/ts/glv.g0);  xygo('Temp', 'fx');
    subplot(324), plot(imu(:,7), imu(:,5)/ts/glv.g0);  xygo('Temp', 'fy');
    subplot(326), plot(imu(:,7), imu(:,6)/ts/glv.g0);  xygo('Temp', 'fz');
    