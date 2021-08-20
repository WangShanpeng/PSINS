function imu = imumeanplot(imu, n, dph)
% SIMU data plot.
%
% Prototype: imu = imumeanplot(imu, n, dph)
% Inputs: imu - SIMU data, the last column is time tag
%         n - mean count
% Output: imu - cumsum IMU oputput
%          
% See also  imuplot, imuresample.

% Copyright(c) 2009-2018, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 04/03/2018
    ts = diff(imu(1:2,end));
    if nargin<3, dph=0; end
    if nargin<2, n=fix(1.0/diff(imu(1:2,end))); end
    imu = [meann(imu(:,1:end-1),n)*n,imu(n:n:end,end)];
    imuplot(imu,dph);
    if size(imu,2)>7,
        imu(:,7:end-1) = imu(:,7:end-1)/n;
        addtemplot(imu(:,7:end));
    end
    subplot(321); title(sprintf('mean time = %.3f (s)', n*ts));