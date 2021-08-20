function imu = imuasyn(imu, tau)
% Set time-asynchrony amont SIMU sensors.
%
% Prototype: imu = imuasyn(imu, tau)
% Inputs: imu - raw SIMU data
%         tau - time-asynchrony parameters, <0 for delay, >0 for leading
% Output: imu - output SIMU data
%
% See also  imuadderr, imuclbt, imuerrset.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/07/2021
    sz = length(tau);
    if sz==1, tau = [0;0;0; tau;tau;tau]; end
    if sz==3&&size(imu,2)==7, tau = [0;0;0; tau]; end
    s = cumsum(imu(:,1:end-1));
    for k=1:size(imu,2)-1
        s(:,k) = interp1(imu(:,end), s(:,k), imu(:,end)+tau(k));
    end
    imu(:,1:end-1) = diffs(s);
    imu = imu(~isnan(sum(imu(:,1:end-1),2)),:);
    