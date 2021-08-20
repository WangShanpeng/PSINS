function [imu, sf] = imuscale(imu, t, w, f)
% Scale gyro(acc) to given angular rate(force) at time t, such that
% [w; f]'*ts = imu(k,1:6) .* sf' .
%
% Prototype: imu = imuscale(imu, t, w, f)
% Inputs: imu - SIMU data array
%         t - specific time
%         w/f - given angular rate(force)
% Outputs: imu - SIMU data array after scaling
%          sf - scale factor
%
% See also  sysclbt, imuinc, imudot, imurot.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 18/01/2021
    if nargin<4, f=[1;1;1]*9.8; end
    if length(w)==1, w=[w;w;w]; end
    if length(f)==1, f=[f;f;f]; end
    wf = [w; f];  sf = wf;
    ts = diff(imu(1:2,end));
    for k=1:6
        idx = find(imu(:,end)>t(k),1,'first');
        wf0 = mean(imu(idx:idx+100,k))/ts;
        sf(k) = wf(k)/wf0;
        imu(:,k) = imu(:,k)*sf(k);
    end
