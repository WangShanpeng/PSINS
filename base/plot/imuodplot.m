function dist = imuodplot(imuod)
% IMU/Odometer plot.
%
% Prototype: imuodplot(imuod)
% Input: imuod - [imu, od_increment, t].
% Output: sv - smooth velocity
%
% See also  imuplot, odplot, dvlplot.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 19/11/2023
global glv
    t = imuod(:,end);  ts = diff(t(1:2));
    dist = cumsum(imuod(:,7:end-1));
    myfig;
	subplot(221), plot(t, imuod(:,1:3)/ts/glv.dps); xygo('w');
	subplot(222), plot(t, imuod(:,4:6)/ts/glv.g0);  xygo('f');
	subplot(223), plot(t, imuod(:,7:end-1)/ts); xygo('V');
	subplot(224), plot(t, dist); xygo('dist / ,');
    title(sprintf('dist = %.3f, ', dist(end,:)));
    

