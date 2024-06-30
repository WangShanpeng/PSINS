function [adir, err] = agdir(acc, g0)
% Acc or gyro direction find.
%
% Prototype: [adir, err] = agdir(acc, g0)
% Inputs: acc - acc data, or gyro data
%         g0 - directon reference array in m/s^s, or rad/s
% Output: adir, err - directon reference in m/s^s & abs(error), or in rad/s
%
% See also  lsclbt.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 01/04/2024
global glv
    [m, n] = size(acc);
    if n>=6, acc=acc(:,4:end); end   % accdir(imu);
    if nargin<2, g0=[glv.g0, 0];  end
    if length(g0)<2, g0=[g0, 0];  end
    adir = zeros(m,3);  err = adir;
    for k=1:m
        for n=1:3
            [err(k,n),idx] = min(abs(abs(acc(k,n))-g0));
            adir(k,n) = sign(acc(k,n))*g0(idx);
        end
    end
    