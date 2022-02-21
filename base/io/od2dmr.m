function [od1, hd] = od2dmr(fname, od, varargin)
% Translate PSINS od array to Inertial Explorer(IE) binary file formats, or vice versa.
%
% Prototype: [od1, hd] = od2dmr(fname, od, varargin)
% Inputs: fname - file name to write
%         od - OD data array.
%         varargin - some extra DMR file header field description
% Outputs: od1 - OD data array.
%          hd - DMR file header.
%
% See also  imu2imr, odfile.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/02/2022
global glv
    if nargin<2  % load DMR file
        fid = fopen(fname, 'r');
        hd.szHdr = fread(fid, [1,8], '*char');
        hd.sHdrSize = fread(fid, 1, '*int16');
        hd.sRecSize = fread(fid, 1, '*int16');
        hd.sValueType = fread(fid, 1, '*int16');
        hd.sMeasType = fread(fid, 1, '*int16');
        hd.sDim = fread(fid, 1, '*int16');
        hd.sRes = fread(fid, 1, '*int16');
        hd.sDistanceType = fread(fid, 1, '*int16');
        hd.sVelocityType = fread(fid, 1, '*int16');
        hd.dScale = fread(fid, 1, 'double');
        hd.szAxisName = fread(fid, [1,48], '*char');
        hd.dWheelSize = fread(fid, 1, 'double');
        hd.lTicksPerRevolution = fread(fid, 1, '*int32');
        hd.cExtra2 = fread(fid, [1,420], '*int8');
        od1 = 1;
        fclose(fid);
        return
    end
    %% save to DMR file
    hd.szHdr = strfill('$DMIRAW',8);
    hd.sHdrSize = int16(512);
    hd.sRecSize = int16(12+8);
    hd.sValueType = int16(1);
    hd.sMeasType = int16(2);
    hd.sDim = int16(1);
    hd.sRes = int16(2);
    hd.sDistanceType = int16(0);
    hd.sVelocityType = int16(1);
    hd.dScale = 1.0;
    hd.szAxisName(1:48) = strfill('PSINS OD translation tools (www.psins.org.cn)',48);
    hd.dWheelSize = 0;
    hd.lTicksPerRevolution = int32(0);
    hd.cExtra2(1:420) = int8(0);
    %%
    for k=1:length(varargin)/2
        hd = setfield(hd, varargin{k}, varargin{2*k});
    end
    sSync = repmat(uint16(hex2dec('ffee')),length(od),1);
    sWeek = repmat(uint16(65535),length(od),1);
    dTime = typecast(od(:,end),'uint16');
    dValue = typecast(od(:,1)/diff(od(1:2,end)),'uint16');
    od1 = [sSync, sWeek, reshape(dTime,4,length(od))', reshape(dValue,4,length(od))'];
    %%
    fid = fopen(fname, 'w');
    fwrite(fid, hd.szHdr, 'char');
    fwrite(fid, hd.sHdrSize, 'int16');
    fwrite(fid, hd.sRecSize, 'int16');
    fwrite(fid, hd.sValueType, 'int16');
    fwrite(fid, hd.sMeasType, 'int16');
    fwrite(fid, hd.sDim, 'int16');
    fwrite(fid, hd.sRes, 'int16');
    fwrite(fid, hd.sDistanceType, 'int16');
    fwrite(fid, hd.sVelocityType, 'int16');
    fwrite(fid, hd.dScale, 'double');
    fwrite(fid, hd.szAxisName, 'char');
    fwrite(fid, hd.dWheelSize, 'double');
    fwrite(fid, hd.lTicksPerRevolution, 'int32');
    fwrite(fid, hd.cExtra2, 'char');
    fwrite(fid, od1', 'uint16');
    fclose(fid);
    
