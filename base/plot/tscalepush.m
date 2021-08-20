function tscalepush(tscale)
% x-axis time scale push.
%
% Prototype: tscalepush(tscale)
% Input: tscale - time scale.
%
% Example:       % imuplot(imu, 't/h');
%    tscalepush('t/h');
%    imuplot(imu/tscaleget());
%    tscalepop();
%
% See also  tscalepop, tscaleget.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/07/2021
    global glv
    if ischar(tscale)
        switch tscale
            case 't/s', tscale=1;
            case 't/m', tscale=60;
            case 't/h', tscale=3600;
            case 't/d', tscale=24*3600;
        end
    end
    glv.tscale = [glv.tscale; tscale];