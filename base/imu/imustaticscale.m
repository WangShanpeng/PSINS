function [imu, sw, sf] = imustaticscale(imu, pos)
% Static high-accuracy SIMU data scale to adequate incremental sampling output.
%
% Prototype: [imu, sw, sf] = imustaticscale(imu, pos)
% Inputs: 
%    imu - high-accuracy raw SIMU data
%    pos - sampling position
% Output: imu - new SIMU data with adequate incremental sampling output
%
% See also  imuscale, imustatic.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 25/08/2022
global glv
    if nargin<2, pos=glv.pos0; end
    ts = diff(imu(1:2,end));
    eth = earth(pos);
    w = norm(mean(imu(:,1:3))); f = norm(mean(imu(:,4:6)));
    sw = glv.wie*ts/w; sf = eth.g*ts/f;
    imu(:,1:6) = [imu(:,1:3)*sw, imu(:,4:6)*sf];
 