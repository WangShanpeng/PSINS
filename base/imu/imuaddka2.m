function imu = imuaddka2(imu, ka2, ka1)
% SIMU acc adding quadratic scale factor errors.
%
% Prototype: imu = imuaddka2(imu, ka2)
% Inputs: imu - raw SIMU data
%         ka2 - acc quadratic coefficient
%         ka1 - if considering linear scale factor compensation, 1 for yes, 0 for no
% Output: imu - output SIMU data added errors
%
% See also  imuadderr, imuerrset, imuclbt, imudeldrift.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 19/08/2022
global glv
    if nargin<3, ka1 = 1; end
    if length(ka2)<3; ka2 = [ka2;ka2;ka2]; end
    ts = imu(2,end)-imu(1,end);
    ka1 = ka1*ka2*glv.g0;
    imu(:,4:6) = [ imu(:,4)+ka2(1)/ts*imu(:,4).^2-ka1(1)*imu(:,4), ...
                   imu(:,5)+ka2(2)/ts*imu(:,5).^2-ka1(2)*imu(:,5), ...
                   imu(:,6)+ka2(3)/ts*imu(:,6).^2-ka1(3)*imu(:,6) ];

