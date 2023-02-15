function [imu, avp0, avp] = imusway(avp0, swayA, swayTau, ts, T, imuerr)
% SIMU sensor incremental outputs on sway base.
%
% Prototype: [imu, avp0] = imusway(avp0, swayA, swayTau, ts, T, imuerr)
% Inputs: avp0 - initial avp0=[att0,vn0,pos0]
%         swayA - angular sway (in rad) & linear motion amplitude (in meter)
%         swayTau - angular sway & linear motion period (in second)
%         ts - SIMU sampling interval
%         T - total sampling simulation time
%         imuerr - SIMU error setting structure array from imuerrset
% Outputs: imu - gyro & acc incremental outputs
%          avp0 - init avp0
%          avp = [att,vn,pos,t] array
%
% Example:
%   glvs;
%   [imu, avp0, avp] = imusway([1;1;10]*glv.deg, [2*glv.deg;1], [5;7;8; 5;6.1;9], 0.01, 100);
%   imuplot(imu);  insplot(avp);
%   avp1 = inspure(imu, avp0);  avpcmpplot(avp, avp1);
%
% See also  imustatic, seawavesimu, swaysimu, seawavedlg, avp2imu.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 20/01/2021
global glv;
    if length(avp0)<6, avp0 = [avp0(1:3); glv.pos0]; end
    if length(avp0)<9, avp0 = [avp0(1:3); zeros(3,1); avp0(4:6)]; end
    swayA = pextend(swayA,0);  swayTau = pextend(swayTau,inf);
    t = (ts:ts:T)';  yaw = avp0(3);
    x = swayA(4)/glv.Re*cos(2*pi/swayTau(4)*t);  y = swayA(5)/glv.Re*cos(2*pi/swayTau(5)*t);
    ap = [ avp0(1)+swayA(1)*cos(2*pi/swayTau(1)*t), ...        % pch
           avp0(2)+swayA(2)*cos(2*pi/swayTau(2)*t), ...        % rll
           avp0(3)+swayA(3)*cos(2*pi/swayTau(3)*t), ...        % yaw
           avp0(7)+(x*sin(yaw)+y*cos(yaw)), ...                % lat
           avp0(8)+(x*cos(yaw)-y*sin(yaw))/cos(avp0(7)), ...   % lon
           avp0(9)+swayA(6)*cos(2*pi/swayTau(6)*t) ];          % hgt
    [imu, avp0, avp] = ap2imu([ap,t]);
    if exist('imuerr', 'var')
        imu = imuadderr(imu, imuerr);
    end
    
function p = pextend(p,val)
    if length(p)==1, p=[p; val]; end
    if length(p)==2, p=[p(1);p(1);p(1); p(2);p(2);p(2)]; end
    if length(p)==3, p=[p; val;val;val]; end
        
