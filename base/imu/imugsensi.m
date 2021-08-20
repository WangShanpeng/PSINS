function imu = imugsensi(imu, gsens, gsens2, gsensX)
% SIMU g-sensitivity correction.
%
% Prototype: imu = imugsensi(imu, gsens, gsens2, gsensX)
% Inputs: imu - raw SIMU data
%         gsens - g-sensitivity 3X3 matrix
%         gsens2 - 2nd order g-sensitivity 3X3 matrix
%         gsensX - cross g-sensitivity 3X3 matrix
% Output: imu - output SIMU data with g-sensitivity correction
%
% See also  imuadderr, imudeldrift.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 24/12/2020
    ts = diff(imu(1:2,end));
    imu(:,1:3) = imu(:,1:3) - imu(:,4:6)*gsens';
    if nargin>2
        imu(:,1:3) = imu(:,1:3) - imu(:,4:6).^2*(gsens2'/ts);
    end
    if nargin>3
        imu(:,1:3) = imu(:,1:3) - [imu(:,4).*imu(:,5),imu(:,4).*imu(:,6),imu(:,5).*imu(:,6)]*(gsensX'/ts);
    end
    