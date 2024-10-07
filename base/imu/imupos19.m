function [imu, att] = imupos19(avp0, ts, Tstop, Tstart)
% SIMU sensor incremental outputs on static base, for 19-position systematic calibration scheme.
%
% Prototype: [imu, att] = imupos19(avp0, ts, Tstop, Tstart)
% Inputs: avp0 - initial avp0=[att0,vn0,pos0]
%         ts - SIMU sampling interval
%         Tstop - stop time between two rotation
%         Tstart - stop time at the first static position
% Output: imu - gyro & acc incremental outputs
%         att - attitude output
%
% Example:
%   [imu, att] = imupos19([[1;2;3]*glv.deg; glv.pos0], 0.01, 20, 70); imuplot(imu);
%
% See also  imustatic, imustatictp, imusway, sysclbt.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/08/2024
global glv;
    if nargin<4, Tstart=70; end
    if nargin<3, Tstart=20; end
    if nargin<4, ts=0.01; end
    Trot = 9;
    paras = [
        1    0,1,0, 90, Trot, Tstart, Tstop
        2    0,1,0, 90, Trot, Tstop, Tstop
        3    0,1,0, 90, Trot, Tstop, Tstop
        4    0,1,0, -90, Trot, Tstop, Tstop
        5    0,1,0, -90, Trot, Tstop, Tstop
        6    0,1,0, -90, Trot, Tstop, Tstop
        7    0,0,1, 90, Trot, Tstop, Tstop
        8    1,0,0, 90, Trot, Tstop, Tstop
        9    1,0,0, 90, Trot, Tstop, Tstop
        10   1,0,0, 90, Trot, Tstop, Tstop
        11   -1,0,0, 90, Trot, Tstop, Tstop
        12   -1,0,0, 90, Trot, Tstop, Tstop
        13   -1,0,0, 90, Trot, Tstop, Tstop
        14    0,0,1, 90, Trot, Tstop, Tstop
        15    0,0,1, 90, Trot, Tstop, Tstop
        16    0,0,-1, 90, Trot, Tstop, Tstop
        17    0,0,-1, 90, Trot, Tstop, Tstop
        18    0,0,-1, 90, Trot, Tstop, Tstop
    ];  paras(:,5) = paras(:,5)*glv.deg;
    att = attrottt(avp0(1:3), paras, ts);
    imu = avp2imu(att,avp0(end-2:end));

