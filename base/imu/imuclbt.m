function imu = imuclbt(imu, clbt, eb, Ka, db)
% IMU simuation by adding calibration errors.
%
% Prototype: imu = imuclbt(imu, clbt)
% Inputs: imu - ideal IMU simulation data
%         clbt - calibration struct
% Output: imu - IMU simulation output
%
% See also  clbtfile, imuadderr, imulvS, sysclbt.

% Copyright(c) 2009-2016, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 16/08/2016
global glv
    ts = diff(imu(1:2,end));
    if nargin<2  % imuclbt(imu)  default as an example
        clbt.Kg = eye(3) - diag([10;20;30]*glv.ppm) + ...
            [0, 10, 20; 30, 0, 40; 50, 60, 0]*glv.sec;
        clbt.eb = [0.1; 0.2; 0.3]*glv.dph;
        clbt.Ka = eye(3) - diag([10;20;30]*glv.ppm) + ...
            [0, 0, 0; 10, 0, 0; 20, 30, 0]*glv.sec;
        clbt.db = [10; 20; 30]*glv.ug;
        clbt.Ka2 = [10; 20; 30]*glv.ugpg2;
        clbt.rx = [1;2;3]/100; clbt.ry = [4;5;6]/100; clbt.rz = zeros(3,1);
        clbt.tGA = 0.001;
    end
    [m, n] = size(clbt);
    if m==3 && n==3  % imuclbt(imu, Kg, eb, Ka, db)
        Kg = clbt;
        if ~exist('eb','var'), eb=zeros(3,1); end
        if ~exist('Ka','var'), Ka=eye(3); end
        if ~exist('db','var'), db=zeros(3,1); end
        imu(:,1:6) = [imu(:,1:3)*Kg',imu(:,4:6)*Ka'] - repmat([eb;db]'*ts,length(imu),1);
        return;
    end
    if ~isfield(clbt, 'Kg'), clbt.Kg = eye(3); end
    if ~isfield(clbt, 'Ka'), clbt.Ka = eye(3); end
    if ~isfield(clbt, 'eb'), clbt.eb = zeros(3,1); end
    if ~isfield(clbt, 'db'), clbt.db = zeros(3,1); end
    if ~isfield(clbt, 'Ka2'), clbt.Ka2 = zeros(3,1); end
    if ~isfield(clbt, 'rx'), clbt.rx = zeros(3,1); end
    if ~isfield(clbt, 'ry'), clbt.ry = zeros(3,1); end
    if ~isfield(clbt, 'rz'), clbt.rz = zeros(3,1); end
    if ~isfield(clbt, 'tGA'), clbt.tGA = 0; end
    timebar(1, length(imu), 'IMU calibration,');
    for k=2:length(imu)
        wb = imu(k,1:3)'/ts; fb = imu(k,4:6)'/ts;
        dwb = (imu(k,1:3)-imu(k-1,1:3))'/ts/ts;
        SS = imulvS(wb, dwb);  fL = SS*[clbt.rx;clbt.ry;clbt.rz];
        fb = fb + fL + clbt.tGA*cross(wb,fb);
        imu(k,4:6) = fb'*ts;
        timebar(1);
    end
    imuerr.dKg = -(clbt.Kg-eye(3)); imuerr.eb = -clbt.eb; imuerr.web = zeros(3,1); imuerr.sqg = zeros(3,1);
    imuerr.dKa = -(clbt.Ka-eye(3)); imuerr.db = -clbt.db; imuerr.KA2 = -clbt.Ka2; imuerr.wdb = zeros(3,1);  imuerr.sqa = zeros(3,1);
    imu = imuadderr(imu, imuerr, ts);
    