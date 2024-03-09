function [imu1, hd] = imu2imr(fname, imu, varargin)
% Translate PSINS IMU array to Inertial Explorer(IE) binary file formats, or vice versa.
%
% Prototype: [imu, hd] = imu2imr(fname, imu, varargin)
% Inputs: fname - file name to write
%         imu - SIMU data array.
%         varargin - some extra IMR file header field description
% Outputs: imu1 - SIMU data array.
%          hd - IMR file header.
%
% See also  od2dmr, imufile, cgfrmvp.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/02/2022
global glv
    if nargin<2  % load IMR file
        fid = fopen(fname, 'r');
        hd.szHeader = fread(fid, [1,8], '*char');
        hd.bIsIntelOrMotorola = fread(fid, 1, 'int8');
        hd.dVersionNumber = fread(fid, 1, 'double');
        hd.bDeltaTheta = fread(fid, 1, 'int32');
        hd.bDeltaVelocity = fread(fid, 1, 'int32');
        hd.dDataRateHz = fread(fid, 1, 'double');
        hd.GyroScaleFactor = fread(fid, 1, 'double');
        hd.AccelScaleFactor = fread(fid, 1, 'double');
        hd.iUtcOrGpsTime = fread(fid, 1, 'int32');
        hd.RcvTimeOrCorrTime = fread(fid, 1, 'int32');
        hd.dTimeTagBias = fread(fid, 1, 'double');
        hd.szImuName = fread(fid, [1,32], '*char');
        hd.reserved1 = fread(fid, [1,4], 'uint8');
        hd.szProgramName = fread(fid, [1,32], '*char');
        hd.tCreate = fread(fid, [1,12], '*char');
        hd.bLeverArmValid = fread(fid, 1, 'int8');
        hd.lXoffset = fread(fid, 1, 'int32');
        hd.lYoffset = fread(fid, 1, 'int32');
        hd.lZoffset = fread(fid, 1, 'int32');
        hd.Reserved = fread(fid, [1,354], '*int8');
        imu1 = fread(fid, [8,inf], 'int32')';
        fclose(fid);
        return;
    end
    %% save to IMR file
    hd.szHeader = strfill('$IMURAW',8);
    hd.bIsIntelOrMotorola = int8(0);
    hd.dVersionNumber = 8.80;
    hd.bDeltaTheta = int32(1);
    hd.bDeltaVelocity = int32(1);
    hd.dDataRateHz = 100;
    hd.GyroScaleFactor = 0.01*glv.sec;
    hd.AccelScaleFactor = 1.0*glv.ug;  % 1*ug*sec
    hd.iUtcOrGpsTime = int32(2);
    hd.RcvTimeOrCorrTime = int32(0);
    hd.dTimeTagBias = 0;
    hd.szImuName(1:32) = strfill('PSINS IMU translation tools',32,0);
    hd.reserved1(1:4) = strfill('PSINS',4,0);
    hd.szProgramName(1:32) = strfill('www.psins.org.cn',32,0);
    hd.tCreate(1:12) = strfill('PSINS',12,0);
    hd.bLeverArmValid = int8(0);
    hd.lXoffset = int32(0);
    hd.lYoffset = int32(0);
    hd.lZoffset = int32(0);
    hd.Reserved(1:354) = int8(0);  hd.Reserved(end) = int8(0);
    hd.dDataRateHz = (length(imu)-1)/(imu(end,end)-imu(1,end));
    %%
    for k=1:length(varargin)/2
        hd = setfield(hd, varargin{k}, varargin{2*k});
    end
    sf = [repmat(hd.GyroScaleFactor,3,1)*glv.deg; repmat(hd.AccelScaleFactor,3,1)];  % deg/s, m/s^2
    for k=1:6
        imu(:,k) = round(cumsum(imu(:,k)/sf(k)));
    end
    imu = [[imu(1,1:6); diff(imu(:,1:6))], imu(:,end)];
    t = typecast(imu(:,end),'int32');
    imu1 = [reshape(t,2,length(imu))', int32(imu(:,1:6))];
    %%
    fid = fopen(fname, 'w');
    fwrite(fid, hd.szHeader, 'char');
    fwrite(fid, hd.bIsIntelOrMotorola, 'int8');
    fwrite(fid, hd.dVersionNumber, 'double');
    fwrite(fid, hd.bDeltaTheta, 'int32');
    fwrite(fid, hd.bDeltaVelocity, 'int32');
    fwrite(fid, hd.dDataRateHz, 'double');
    fwrite(fid, hd.GyroScaleFactor, 'double');
    fwrite(fid, hd.AccelScaleFactor, 'double');
    fwrite(fid, hd.iUtcOrGpsTime, 'int32');
    fwrite(fid, hd.RcvTimeOrCorrTime, 'int32');
    fwrite(fid, hd.dTimeTagBias, 'double');
    fwrite(fid, hd.szImuName, 'char');
    fwrite(fid, hd.reserved1, 'uint8');
    fwrite(fid, hd.szProgramName, 'char');
    fwrite(fid, hd.tCreate, 'char');
    fwrite(fid, hd.bLeverArmValid, 'int8');
    fwrite(fid, hd.lXoffset, 'int32');
    fwrite(fid, hd.lYoffset, 'int32');
    fwrite(fid, hd.lZoffset, 'int32');
    fwrite(fid, hd.Reserved, 'int8');
    fwrite(fid, imu1', 'int32');
    fclose(fid);
    
