function [att0, attk, xkpk] = aligngps(imu, gps, qnb, phi0, imuerr, wvn, isfig)
% SINS initial align uses 15-state SINS/GPS Kalman filter with vn/pos as measurement.
%
% Prototype: [att0, attk, xkpk] = aligngps(imu, gps, qnb, phi0, imuerr, wvn, isfig)
% Inputs: imu - IMU data
%         gps - GPS array = [vnGPS, posGPS, t]
%         qnb - coarse attitude quaternion
%         phi0 - initial misalignment angles estimation
%         imuerr - IMU error setting
%         wvn - velocity measurement noise (3x1 vector)
%         isfig - figure flag
% Outputs: att0 - attitude align result
%         attk, xkpk - for debug
%
% Example:
%	avp0 = [[0;0;0]; zeros(3,1); glv.pos0];
%	imuerr = imuerrset(0.03, 100, 0.001, 10);
%	imu = imustatic(avp0, 0.1, 600, imuerr);  gps = gpssimu([avp0(4:9);600], 0.1, 1);
%	phi = [.5; .5; 5]*glv.deg;
%	wvn = vperrset([0.1;0.3],[1,3]);
%	[att0, attk, xkpk] = aligngps(imu, gps, [1;2;3]*glv.deg, phi, imuerr, wvn);
%
% See also  alignvn, sinsgps.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/11/2021
global glv
    if size(gps,2)<=5, gpspos_only=1; else, gpspos_only=0; end 
    if nargin<4,  phi0 = [1.5; 1.5; 3]*glv.deg;  end
    if nargin<5,  imuerr = imuerrset(0.01, 100, 0.001, 1);  end
    if nargin<6
        if gpspos_only==1, wvn=poserrset([10,30]);
        else, wvn=vperrset([0.1;0.3],[10,30]); end
    end
    if nargin<7,  isfig = 1; end
    if length(qnb)==3, qnb=a2qua(qnb); end  %if input qnb is Eular angles.
    [nn, ts, nts] = nnts(2, diff(imu(1:2,end)));
    ins = insinit([q2att(qnb);gps(1,1:6)'], ts);
    davp = avperrset(phi0/glv.min, 1, 10);
    [avp, xkpk, zkrk, sk, ins, kf] = sinsgps(imu, gps, ins, davp, imuerr, [0;0;0;0], [0;0], wvn, 0, 0, 'avped', 0);
    att0 = avp(end,1:3);
    attk = avp(:,[1:3,end]);
    if isfig==1
        insplot(avp);
        kfplot(xkpk);
    end
    
