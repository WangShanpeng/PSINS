function m = imumean(imu, t0, t1, ts)
% SIMU mean angular rate & specific force within [t0,t1].
%
% Prototype: m = imumean(imu, t0, t1, ts)
% Inputs: imu - SIMU data, the last column is time tag
%         t0 - mean start/end time
%         ts - sampling interval
% Output: m - mean w/f out
%          
% See also  imuplot, imumeanplot.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/06/2023
    if nargin<4, ts = diff(imu(1:2,end)); end
    if length(t1)==1, t1=t0+t1; end  % dt=t1
%     t0 = t0-imu(1,end); t1 = t1-imu(1,end);
    m = zeros(length(t0),size(imu,2)-1);
    for k=1:length(t0)
        imui = datacut(imu, t0(k), t1(k));
        m(k,:) = mean(imui(:,1:end-1));
%         m(k,:) = mean(imu(fix(t0(k)/ts):fix(t1(k)/ts), 1:6));
    end
    m = m/ts;  % in rad/s, m/ss
    