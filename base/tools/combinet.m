function  [i1, i2, dt] = combinet(t1, t2)
% Combine times between time t1 & t2, such that: dt = t1(i1)-t2(i2).
% Note: t1 should be higher frequency, and t2 lower frequency.
%
% Prototype: [i1, i2, dt] = combinet(t1, t2)
% Inputs: t1, t2 - two time tags
% Outputs: i1, i2, dt - index & dt
%
% Example:
%   1. [i1, i2, dt] = combinet((1:10)', [-1;2;5.6]);
%   2. [i1, i2, dt] = combinet(imu(:,end)-gps(1,end), gps(:,end)-gps(1,end));
%
% See also  combinedata, timeunion, imugpssyn, gett, datacut.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/01/2020
    ti = interp1(t1, t1, t2, 'nearest');
    [na, i1, i2] = intersect(t1, ti);
    dt = t1(i1)-t2(i2);
%     figure, plot(t1, t1*0, '-', t2, t2*0+1, '--', t1(i1), dt+0.5, 'o');  xygo('dt');