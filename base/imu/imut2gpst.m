function [imut, err] = imut2gpst(imutf, nHz)
% Correct IMU sampling time to GPS time.
%
% Inputs: imutf - [imut, gpsflag], gpsflag>0 for gps valid, else gpsflag=0
%         nHz - gps sampling frequency
% Outputs: imut - new imu time tag
%          err - error in ppm
%
% See also  imugpssyn.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/09/2021
    idx = imutf(:,2)>0;
    gpst = imutf(idx,1);
    if nargin<2, nHz=round(1/mean(diff(gpst))); end
    dt = round(diff(gpst)*nHz)/nHz;
    gpst = cumsum([gpst(1); dt]);
    pp = polyfit(gpst, imutf(idx,1)-gpst, 2);
    imut = imutf(:,1) + polyval(pp, imutf(:,1));
    err = (mean(imut)/mean(imutf(:,1))-1)*1e6;
    myfig;
    subplot(211), plot(gpst(2:end), [diff(imutf(idx,1)),dt]); xygo('diff-gpst / s');
    subplot(212), plot(gpst, imutf(idx,1)-gpst, imutf(:,1), polyval(pp, imutf(:,1))); xygo('cum err / s');
    title(sprintf('%.2f ppm', err)); 

