function t = imustaticplot(imu, minT, wth, fth)
% SIMU static data plot.
%
% Prototype: t = imustaticplot(imu, minT, wth, fth)
% Inputs: imu - SIMU data, the last column is time tag
%         minT - min T interval 
%         wth,fth - angular rate/specific force threshold
% Output: t - time interval
%          
% See also  imuplot, imutplot, imutemplot, imuresample.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/08/2023
global glv
    if nargin<4, fth=0.01*glv.g0; end
    if nargin<3, wth=0.1*glv.dps; end
    if nargin<2, minT=10; end
    ts = diff(imu(1:2,end));
    w = normv(imu(:,1:3));  f = normv(imu(:,4:6));
    idx = (w<wth*ts); % & (f<fth*ts);
    w(:) = 0; w(idx) = 1;
    w = [1; diff(w)];  w(end)=-1;
    idx0 = find(w>0);   idx1 = find(w<0);
    t = [];  imu1 = [];
    for k=1:min(length(idx0),length(idx1))
        if idx1(k)-idx0(k)>minT/ts
            t = [t; [imu(idx0(k),end), imu(idx1(k),end)]];
            imu1 = [imu1; imu(idx0(k):idx1(k),:)];
        end
    end
    imuplot(imu1);
