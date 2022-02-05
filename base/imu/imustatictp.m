function [imu, att] = imustatictp(avp0, ts, T, wz, az, imuerr)
% SIMU sensor incremental outputs on static base, for two-position alignment simulation.
%
% Prototype: imu = imustatictp(avp0, ts, T, wz, az, imuerr)
% Inputs: avp0 - initial avp0=[att0,vn0,pos0]
%         ts - SIMU sampling interval
%         wz - positive rotation angular rate, in rad/s
%         az - rotation angle, in rad
%         imuerr - SIMU error setting structure array from imuerrset
% Output: imu - gyro & acc incremental outputs
%         att - attitude output
%
% Example:
%   imuerr = imuerrset(0.01,100,0.001,10, 0.001,1000,10,1000, 10,10,10,10);
%   [imu, att] = imustatictp([[1;2;10]*glv.deg, glv.pos0], 0.01, 300, -10*glv.dps, 180*glv.deg, imuerr); imuplot(imu);
%
% See also  imustatic, imusway.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/11/2021
global glv;
    if ~exist('imuerr', 'var'), imuerr = imuerrset(0,0,0,0); end
    if ~exist('az', 'var'), az = 180*glv.deg; end;
    if ~exist('wz', 'var'), wz = 10*glv.dps; end
    if length(avp0)<6, avp0 = [avp0(1:3); glv.pos0]; end
    if length(avp0)<9, avp0 = [avp0(1:3); zeros(3,1); avp0(4:6)]; end
    Tturn = abs(az/wz);
    if Tturn>T/3, Tturn=T/3; end
    
    paras = [  1    0,0,1,   az/glv.deg, Tturn, T/2-Tturn/2, T/2-Tturn/2];
    paras(:,5) = paras(:,5)*glv.deg;
    att = attrottt(avp0(1:3), paras, ts, 0);
    len = length(att);
    avp = [att(:,1:3),zeros(len,3),repmat(avp0(7:9)',len,1),att(:,4)];
    imu = imuadderr(avp2imu(avp), imuerr);
    att = avp(:,[1:3,end]);
    insplot(att);

%     xxx = [];
%     seg = trjsegment(xxx, 'init',         0);
%     seg = trjsegment(seg, 'uniform',      T/2-Tturn/2);
%     seg = trjsegment(seg, 'turnleft',     Tturn, wz/glv.dps);
%     seg = trjsegment(seg, 'uniform',      T/2-Tturn/2);
%     trj = trjsimu(avp0, seg.wat, ts, 1);
%     imu = imuadderr(trj.imu, imuerr, ts);
%     att = trj.avp(:,[1:3,end]);


