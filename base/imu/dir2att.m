function [att, C] = dir2att(dir0, yaw0, imu)
% Trans SIMU direction string or number to attitude.
%
% Prototype: att = dir2att(dir0, yaw0)
% Inputs: dir0 - SIMU direction at the first epoch, w.r.t. R-F-U
%         yaw0 - relative yaw at the fist epoch
%         imu - SIMU array
% Output: att - =[pitch; roll; yaw]
%
% Example
%            东天南，地东南，南东天， 西南天，西地南，天西南，北西天，西南天
%    rot = [ 1 5 2;  6 1 2; 2 1 5;  3 2 5;  3 6 2;  5 3 2; 4 3 5; 3 2 5 ];
%    dir2att(rot(8,:))/glv.deg

% See also  cumwie, wierfu, lsgsen.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/04/2024
    if nargin<2; yaw0=0; end
    if isnumeric(dir0),  % 123456 for RBLFUD / ESWNUD
        if length(dir0)==2 || length(dir0)==4; alnT=dir0(end); dir0=dir0(1:end-1); end    % dir0=[dir0;alnT]
        if length(dir0)==1, h1=fix(dir0/100); h2=fix((dir0-h1*100)/10); h3=mod(dir0,10);  % ie dir0=145
        else, h1=dir0(1); h2=dir0(2); h3=dir0(3);                                         % ie dir0=[1 4 5]
        end
        D=[1,0,0; 0,-1,0; -1,0,0; 0,1,0; 0,0,1; 0,0,-1]; Cnb=D([h1;h2;h3],:)';
        att = m2att(a2mat([0;0;yaw0])*Cnb);
        return;
        str='RBLFUD'; dir0=str([h1,h2,h3]);
    end
%     dir0 = upper(dir0);
%     xdir = dir0(1); ydir = dir0(2);  rll = 0;
%     switch ydir
%         case {'R','B','L','F', 'E','S','W','N'}
%             yaw = frbl2yaw(ydir);  pch=0;
%         case 'U'
%             yaw = frbl2yaw(xdir)+0.5*pi; pch=0.5*pi;
%         case 'D'
%             yaw = frbl2yaw(xdir)-0.5*pi; pch=-0.5*pi;
%     end
%     if exist('alnT','var')
%         att = alignsb(imu(1:fix(alnT/ts),:), 0);  pch=att(1); rll=att(2);  % use the first alnT*sec data to get level angles
%     end
%     att = [pch; rll; yaw+yaw0];
    
% function yaw = frbl2yaw(rblf)
%     switch rblf
%         case {'F','N'}, yaw=0;
%         case {'R','E'}, yaw=-0.5*pi;
%         case {'B','S'}, yaw=pi;
%         case {'L','W'}, yaw=0.5*pi;
%     end
    