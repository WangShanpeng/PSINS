function g = imu2g(imu, t1, t2)
% Using SIMU to calculate local gravity magnitude (maybe not accurate).
%
% Prototype: g = imu2g(imu, t1, t2)
% Inputs: imu - SIMU data
%         t1, t2 - using data interval from t1 to t2, default the whole data
% Output: g - local gravity magnitude
%
% See also  imuclbt.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/1/2022
    if nargin<3, t2=inf; end
    if nargin<2, t1=-inf; end
    imu = datacut(imu, t1, t2);
    g = mean(normv(imu(:,4:6)))/diff(imu(1:2,end));
