function is = hrgbi(gyro, ts, isfig)
% Hemispherical Resonator Gyro bias instability calculate using 100s smoothing std.
%
% Prototype: is = hrgbi(gyro, ts, isfig)
% Inputs: gyro - gyro data
%         ts - sampling interval
%         isfig - figure flag
% Output: is - instability value
%
% Example:
%   is = hrgbi(randn(36*1000,1), 1);
%
% See also: avar.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/11/2022
global glv
    n = size(gyro,2);
    if n>3, ts=diff(gyro(1:2,end)); gyro=gyro(:,1:3)/ts/glv.dph; n=3; end  % is = hrgis(imu);
    gyro = meann(gyro, fix(100/ts));
    is = gyro(1:end-35,:);
    for k=1:length(is)
        is(k,:) = std(gyro(k:k+35,:));
    end
    if nargin<3, isfig=1; end
    xyz = 'xyz';
    if isfig==1
        myfig;
        for k=1:n
            subplot(n,2,2*k-1); plot(gyro(:,k)); xygo('t / 100s', [xyz(k),'-gyro smoothing / ( \circ/h )']);
            subplot(n,2,2*k);   plot(is(:,k));   xygo('t / 100s', [xyz(k),'-gyro instability / ( \circ/h )']);
        end
    end