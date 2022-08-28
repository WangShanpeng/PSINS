function [imu, clbt] = imutclbt(imu, clbt)
% IMU calibration with temperature.
%
% Prototype: imu = imutclbt(imu, clbt)
% Inputs: imu - IMU before calibration = [wm, vm, temperature, t]
%         clbt - calibration struct
% Output: imu - IMU output after calibration 
%         clbt - new calibration struct
%
% See also  imuclbt, clbt2array, array2clbt, imutplot.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 12/08/2022
    for k=1:6, imu(:,k)=imu(:,k)*clbt.sf(k); end
    timebar(1, length(imu), 'IMU calibration,');
    ts = imu(2,end)-imu(1,end);
    for k=2:length(imu)
        idx = rem(k,37)+1;
        a = clbt.TempArray(idx,1) + polyval(clbt.TempArray(idx,2:end), imu(k,7));
        clbt = array2clbt(a, clbt, idx);
        imu(k,1:6) = [imu(k,1:3)*clbt.Kg'-clbt.eb'*ts, imu(k,4:6)*clbt.Ka'-clbt.db'*ts-clbt.Ka2'.*imu(k,4:6).^2/ts];
        wb = imu(k,1:3)'/ts; fb = imu(k,4:6)'/ts;
        dwb = (imu(k,1:3)-imu(k-1,1:3))'/ts/ts;
        SS = imulvS(wb, dwb);  fL = SS*[clbt.rx;clbt.ry;clbt.rz];
        fb = fb - fL + clbt.tGA*cross(wb,fb);
        imu(k,4:6) = fb'*ts;
        timebar(1);
    end
