function imu = imuclbt(imu, clbt, eb, Ka, db)
% IMU calibration.
%
% Prototype: imu = imuclbt(imu, clbt)
% Inputs: imu - IMU before calibration 
%         clbt - calibration struct
% Output: imu - IMU output after calibration 
%
% See also  clbtfile, imuadderr, imulvS, sysclbt, lsclbt, imutclbt, imuasyn.

% Copyright(c) 2009-2016, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 16/08/2016
global glv
    ts = diff(imu(1:2,end));
    if nargin<2  % imuclbt(imu)  default as an example
        clbt.sf = [1; 1; 1; 1; 1; 1];
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
        if isempty(Kg), Kg=eye(3); elseif length(Kg)==1, Kg=diag([Kg,Kg,Kg]); elseif size(Kg,2)==1, Kg=diag(Kg); end   % [], scale or 3x1 diag vector
        if isempty(eb), eb=zeros(3,1); elseif length(eb)==1, eb=[eb;eb;eb]; end  % [] or scale
        if isempty(Ka), Ka=eye(3); elseif length(Ka)==1, Ka=diag([Ka,Ka,Ka]); elseif size(Ka,2)==1, Ka=diag(Ka); end
        if isempty(db), db=zeros(3,1); elseif length(db)==1, db=[db;db;db]; end
        imu(:,1:6) = [imu(:,1:3)*Kg',imu(:,4:6)*Ka'] - repmat([eb;db]'*ts,length(imu),1);
        return;
    end
    if ~isfield(clbt, 'sf'), clbt.sf = [1;1;1;1;1;1]; end
    if ~isfield(clbt, 'Kg'), clbt.Kg = eye(3); end
    if ~isfield(clbt, 'Ka'), clbt.Ka = eye(3); end
    if ~isfield(clbt, 'eb'), clbt.eb = zeros(3,1); end
    if ~isfield(clbt, 'db'), clbt.db = zeros(3,1); end
    if ~isfield(clbt, 'Ka2'), clbt.Ka2 = zeros(3,1); end
    if ~isfield(clbt, 'Kap'), clbt.Kap = zeros(3,1); end
    if ~isfield(clbt, 'rx'), clbt.rx = zeros(3,1); end
    if ~isfield(clbt, 'ry'), clbt.ry = zeros(3,1); end
    if ~isfield(clbt, 'rz'), clbt.rz = zeros(3,1); end
    if ~isfield(clbt, 'tGA'), clbt.tGA = 0; end
    for k=1:6, imu(:,k)=imu(:,k)*clbt.sf(k); end
    imuerr.dKg = (clbt.Kg-eye(3)); imuerr.eb = -clbt.eb; imuerr.web = zeros(3,1); imuerr.sqg = zeros(3,1);
    imuerr.dKa = (clbt.Ka-eye(3)); imuerr.db = -clbt.db; imuerr.wdb = zeros(3,1); imuerr.sqa = zeros(3,1);
    if isfield(clbt, 'Ka2'), imuerr.Ka2 = -clbt.Ka2; end
    if isfield(clbt, 'Kap'), imuerr.Kap = -clbt.Kap; end
    if isfield(clbt, 'gSens'), imuerr.gSens = -clbt.gSens; end
    imu = imuadderr(imu, imuerr);
    if norm(clbt.rx)>0 || norm(clbt.ry)>0 || norm(clbt.rz)>0 || abs(clbt.tGA)>0
        timebar(1, length(imu), 'IMU calibration,');
        for k=2:length(imu)
            wb = imu(k,1:3)'/ts; fb = imu(k,4:6)'/ts;
            dwb = (imu(k,1:3)-imu(k-1,1:3))'/ts/ts;
            SS = imulvS(wb, dwb);  fL = SS*[clbt.rx;clbt.ry;clbt.rz];
            fb = fb - fL - clbt.tGA*cros(wb,fb);
            imu(k,4:6) = fb'*ts;
            timebar(1);
        end
    end
    