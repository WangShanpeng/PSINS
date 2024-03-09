function [imu, eb, db] = imudeldrift(imu, t0, t1, avp, yaw0)
% For MEMS-grade IMU and using static-base condition within time interval
% [t0,t1], correct the gyro output by deleting their bias.
%
% Prototype: [imu, eb] = imudeldrift(imu, t0, t1, avp, yaw0)
% Inputs: imu - raw SIMU data
%         t0,t1 - assuming a static-base condition time interval
% Output: imu - new SIMU data with gyro bias deleted
%
% See also  delbias, imuadderr, imurepair, imuresample.

% Copyright(c) 2009-2017, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 25/06/2017, 30/11/2020
    ts = diff(imu(1:2,end));
    if nargin>4   % [imu, eb] = imudeldrift(imu, t0, t1, pos, yaw0);
        pos = avp;
        att = alignsb(datacut(imu,t0,t1),pos,0);
        avp = [att(1:2);yaw0; zeros(3,1); pos];
        wbib = imustatic(avp, 1, 1);
        wbib = wbib(1,1:3)'*ts;
    end
    if nargin>3  % [imu, eb] = imudeldrift(imu, t0, t1, avp);
        if size(avp,2)>1,  avp = getat(avp,t0);   end
        wbib = imustatic(avp, 1, 1);
        wbib = wbib(1,1:3)'*ts;
    else
        wbib = zeros(3,1);
    end
    if length(t0)==1  % [imu, eb] = imudeldrift(imu, t0, t0+10);
        if nargin<3, t1=t0+10; end
        idx0 = find(imu(:,end)>t0,1);
        idx1 = find(imu(:,end)>t1,1);
        eb = mean(imu(idx0:idx1,1:3),1)'-wbib;
        db = zeros(3,1);
    else
        if length(t0)==3,   % [imu, eb] = imudeldrift(imu, eb);
            eb = t0*ts; db = [0;0;0];
        elseif length(t0)==6,  % [imu, eb, db] = imudeldrift(imu, [eb;db]);
            eb = t0(1:3)*ts; db = t0(4:6)*ts;
        else  % [imu, eb, db] = imudeldrift(imu, avp, t0);
            ebdb = getat(t0(:,10:end),t1); %t0=avp;
            eb = ebdb(1:3)*ts; db = ebdb(4:6)*ts;
        end
    end
    imu(:,1:6) = [imu(:,1)-eb(1), imu(:,2)-eb(2), imu(:,3)-eb(3)...
                  imu(:,4)-db(1), imu(:,5)-db(2), imu(:,6)-db(3) ];
    eb = eb/ts; db = db/ts;
    

