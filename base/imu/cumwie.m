function [cwie, cwiek] = cumwie(imu, dir0, pos0, yaw0, isfig)
% Angular cumulative sum of the Earth rotation expressed in body-frame, 
% useful for SIMU separating calibration.
%
% Prototype: [cwie, cwiek] = cumwie(imu, dir0, lat0, yaw0, isfig)
% Inputs: imu - the user's raw SIMU data
%         dir0 - SIMU direction at the first epoch, w.r.t. R-F-U
%         pos0 - test position
%         yaw0 - relative yaw at the fist epoch
%         isfig - figure flag
% Output: cwie, cwiek - angular cumulative sum in rad
%
% See also  lsclbt, wierfu, imurot.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/04/2024
    ts = diff(imu(1:2,end));
    if nargin<4; yaw0=0; end
    if length(pos0)<3, pos0=[pos0(1);0;0]; end
    if isempty(dir0)  % cwie = cumwie(imu, [], pos0, alnT, isfig);
        alnT = yaw0;  yaw0=0;
        att = alignsb(imu(1:fix(alnT/ts),:), pos0);  pch=att(1); rll=att(2); yaw=att(3);  % use the first 1sec data to get level angles
    else
        if isnumeric(dir0),  % 123456 for RBLFUD
            if length(dir0)==1, h1=fix(dir0/100); h2=fix((dir0-h1*100)/10); h3=mod(dir0,10);  % 145
            else, h1=dir0(1); h2=dir0(2); h3=dir0(3);                                         % [1 4 5]
            end
            str='RBLFUD'; dir0=str([h1,h2,h3]);
        end
        dir0 = upper(dir0);
        xdir = dir0(1); ydir = dir0(2); zdir = dir0(3);
        pch = 0; rll = 0;
        switch ydir
            case {'R','B','L','F'}
                yaw = frbl2yaw(ydir);
            case {'U','D'}
                yaw = frbl2yaw(xdir)-0.5*pi;
                if ydir=='U', pch=0.5*pi; else, pch=-0.5*pi; end
        end
    end
    qnb = a2qua([pch; rll; yaw+yaw0]);
    [wnie, g] = wnieg(pos0);  wniets = wnie*ts;  cwie = zeros(3,1);
    cwiek = imu(:,4:7);  attk = imu(:,4:7);
    for k=1:length(imu)
        qnb = qupdt2(qnb, imu(k,1:3)', wniets);
        cwie = cwie + qmulv(qconj(qnb),wniets);
        cwiek(k,1:3) = cwie';
        attk(k,1:3) = q2att(qnb)';
    end
    if nargin<5, isfig=0; end
    if isfig, insplot(attk,'a'); end
    
function yaw = frbl2yaw(rblf)
    switch rblf
        case 'F', yaw=0;
        case 'R', yaw=0.5*pi;
        case 'B', yaw=pi;
        case 'L', yaw=1.5*pi;
    end
    