function [imu, eth] = imustatic(avp0, ts, T, imuerr)
% SIMU sensor incremental outputs on static base.
%
% Prototype: [imu, eth] = imustatic(avp0, ts, T, imuerr)
% Inputs: avp0 - initial avp0=[att0,vn0,pos0]
%         ts - SIMU sampling interval
%         T - total sampling simulation time
%         imuerr - SIMU error setting structure array from imuerrset
% Outputs: imu - gyro & acc incremental outputs
%          eth - the Earth parameters corresponding to avp0
%
% Example:
%   imuerr = imuerrset(0.01,100,0.001,10, 0.001,1000,10,1000, 10,10,10,10);
%   avp0 = avpset([0;0;0]*glv.deg, 0, glv.pos0, 0);
%   imu = imustatic(avp0, 1, 300, imuerr);
%
% See also  imustatictp, imusway, imuerrset, trjsimu, trjunilat, insupdate.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/08/2013, 10/01/2014
global glv;
    if ~exist('imuerr', 'var'), imuerr = imuerrset(0,0,0,0); end
    if length(avp0)<3, avp0 = [0; 0; avp0(1)]; end
    if length(avp0)==3, avp0 = [zeros(6,1);avp0]; end
    if length(avp0)<6, avp0 = [avp0(1:3); glv.pos0]; end
    if length(avp0)<9, avp0 = [avp0(1:3); zeros(3,1); avp0(4:6)]; end
    Cbn = a2mat(avp0(1:3))';
    eth = earth(avp0(7:9), avp0(4:6));
    imu = [Cbn*eth.wnie; -Cbn*eth.gn]';
    if nargin<2,  ts = 1; T = ts;  end
    if nargin<3,  T = ts;  end
    len = round(T/ts);
    imu = repmat(imu*ts, len, 1);
    imu(:,7) = (1:len)'*ts;
    if nargin==4
        imu = imuadderr(imu, imuerr);
    end

