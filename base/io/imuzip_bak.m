function [imu, mst] = imuzip_bak(imu, bytes, istest)
% IMU double-data zip to 1/2 bytes array, or reverse.
%
% Prototype: [imu, mst] = imuzip(imu, bytes, istest)
% Inputs: imu - IMU data input
%         tytes - =1 for 1bytes, =2 for 2bytes
%         istest - test for zip error
% Outputs: imu - IMU data output
%          mst - struct [mst.mean, mst.scale, mst.ts, mst.t0]
%
% Example1:
%    imuf = imustatic(avpset(0,0,glv.pos0), 0.01, 100, imuerrset(10,1000,1,10));
%    [imu8, mst] = imuzip(imuf, 1, 1);
%    imuf1 = imuzip(imu8, mst);  imuplot(imuf,imuf1);
% Example2:
%    imuf = imustatic(avpset([5,10,30],0,glv.pos0), 0.01, 100, imuerrset(10,1000,1,10));
%    [imu16, imu8] = imuzip(imuf);
%    imuf16 = imuzip(imu16);  imuf8 = imuzip(imu8);
%    imuplot(imuf,imuf16);  %   plotn(cumsum(imuf16-imuf8));
%
% See also  imufile.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/11/2024
    if nargin==1    % imu = imuzip(imu)  % only for static z-axis up & 100Hz & t0=ts
        mst = default_mst();
        if ~isinteger(imu)  % zip
            imu(:,6) = imu(:,6)-mst.mean(6);
            imu = cumsum(imu);
            for k=1:6, imu(:,k) = imu(:,k)/mst.scale(k); end
            imu = int64(imu(:,1:6));
            imu = diff([int64(zeros(1,6)); imu]);
            mst.max = max(max(imu)); mst.min = min(min(imu));
            imu = int16(imu);  mst = int8(imu);
            return;
        else    % unzip
            bytes = mst;
        end
    end
    if nargin<3, istest=0; end
    if ~exist('bytes','var'), bytes=1; end
    if isstruct(bytes)  % imu = imuzip(imu, mst)
        mst = bytes;
        [m,n] = size(imu);
        imu1 = zeros(m,n+1);
        for k=1:size(imu,2)
            imu1(:,k) = double(imu(:,k))*mst.scale(k)+mst.mean(k);
        end
        imu1(:,end) = (0:m-1)'*mst.ts + mst.t0;
        imu = imu1;
    else  %  [imu, mst] = imuzip(imu, 1/2, 0/1);
        if istest==1, imu0=imu; end
        mst.t0 = imu(1,end);
        mst.ts = mean(diff(imu(:,end)));  % equal interval
        mst.mean = mean(imu(:,1:end-1));
        imu(:,end) = [];  n=size(imu,2);
        for k=1:n, imu(:,k) = imu(:,k)-mst.mean(k); end
        if bytes==2, s=2^15; elseif bytes==1, s=2^7; end
        mst.scale = (max(imu)-min(imu))/s;
        imu = cumsum(imu);
        for k=1:n, imu(:,k) = imu(:,k)/mst.scale(k); end
        imu = int64(imu);
        imu = diff([int64(zeros(1,n)); imu]);
        mst.max = max(max(imu)); mst.min = min(min(imu));
        if bytes==2, imu=int16(imu); elseif bytes==1, imu=int8(imu); end
        if istest==1
            myfig, plot(imu0(:,end), imuzip(imu,mst)-imu0);  xygo('Cumsum Error');
        end
    end

function mst = default_mst()
global glv
    mst.ts = 0.01;  mst.t0 = mst.ts;
    mst.scale = [[1,1,1]*1*glv.dps, [1,1,1]*10*glv.deg*glv.g0]*0.01/2^7;  % default scale
    % rgyro=scale(1:3)/glv.sec, racc=scale(4:6)/glv.gxs
    mst.mean = [0,0,0, 0,0,glv.g0]*mst.ts;
    