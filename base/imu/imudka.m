function imu = imudka(imu, dka)
% IMU gyro scale modify.
%
% Prototype: imu = imudka(imu, dka)
% Inputs: imu - IMU before calibration 
%         dka - scale factors
% Output: imu - IMU output after calibration 
%
% See also  imudkg, imuclbt, imuadderr, imuscale.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 18/12/2022
    if length(dka)==1
        imu(:,6) = (1+dka)*imu(:,6);
    else
        for k=4:6, imu(:,k) = (1+dka(k))*imu(:,k); end
    end
    