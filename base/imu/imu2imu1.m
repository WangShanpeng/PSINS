function imu1 = imu2imu1(imu2)
% Trans 2-records/1-row IMU to 1-record/1-row.
%
% Prototype: imu1 = imu2imu1(imu2)
% Input: 
%    imu2 - = [ wm1, vm1, wm2, vm2, t1
%               wm3, vm3, wm4  vm4, t3
%               ... ]
% Output: imu1 - = [ wm1, vm1, t1
%                    wm2, vm2, t2
%                    wm3, vm3, t3
%                    wm4  vm4, t4
%                    ... ]
% See also  imuresample.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 16/08/2021
    ts = diff(imu2(1:2,end));
    imu1 = zeros(length(imu2)*2,7);
    imu1(1:2:end,:) = imu2(:,[1:6,end]);
    imu1(2:2:end,:) = [imu2(:,[7:12]),imu2(:,end)+ts/2];