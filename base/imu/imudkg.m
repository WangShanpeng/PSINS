function imu = imudkg(imu, dkg)
% IMU gyro scale modify.
%
% Prototype: imu = imudkg(imu, dkg)
% Inputs: imu - IMU before calibration 
%         dkg - scale factors
% Output: imu - IMU output after calibration 
%
% See also  imudka, imuclbt, imuadderr, imuscale.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 18/12/2022
    if length(dkg)==1
        imu(:,3) = (1+dkg)*imu(:,3);
    else
        for k=1:3, imu(:,k) = (1+dkg(k))*imu(:,k); end
    end
    