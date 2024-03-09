function  [data, dataO] = combinedata(data1, data2)
% Combine data between data1 & data2, where data1 = [data11, t1], data2 = [data22, t2]
% such that: data = [data11, t1, data22, dt], where dt = t1-t2.
% Note: data1 should be higher frequency, and data2 lower frequency.
%
% Example:
%   data = combinedata(imu, gps);
%
% See also  combinet, timeunion, imugpssyn, gett, datacut.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/02/2020
    data2 = datacut(data2, data1(1,end), data1(end,end));
    t2 = interp1(data1(:,end), data1(:,end), data2(:,end), 'nearest');
    [na, i1, i2] = intersect(data1(:,end), t2);
    data = [data1, zeros(size(data1,1),size(data2,2))];
    data(i1,end-size(data2,2)+1:end) = [data2(i2,1:end-1), data1(i1,end)-data2(i2,end)];
    if nargout==2
        dataO = data;  dataO(:,size(data1,2))=[];  dataO(:,end)=data(:,size(data1,2));
    end

