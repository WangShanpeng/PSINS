function od1 = odrepair(od, imut, badValue)
% Odometer data repair.
%
% Prototype: od = odrepair(od, imut)
% Inputs: od - [od_increment, t].
%         imut - imu time stamp for interpolation
%         badValue - bad value to replace
% Output: od1 - od repaired & interpolated output
%
% See also  odplot.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/08/2021
    if nargin<3,  badValue = -100;  end
    od = reminct(od);
    ts = (od(end,end)-od(1,end))/length(od);  % od mean sampling interval
    if nargin>1  % set od(:,end) to imut
        if imut(1,end)<od(1,end)
           od = [[ones(1,size(od,2)-1)*badValue, imut(1,end)]; od];
        end
        if imut(end,end)>od(end,end)
           od = [od; [ones(1,size(od,2)-1)*badValue, imut(end,end)]];
        end
    end
    dt = diff(od(:,end));  % find bad points, if time interval > 1.5*ts
    idx = find(dt>1.5*ts);
    for k=length(idx):-1:1
        ti = od(idx(k),end)+ts:ts:od(idx(k)+1,end)-0.5*ts;
        if ~isempty(ti)
            odi = ones(length(ti),size(od,2)-1)*badValue;
            od = [od(1:idx(k),:); [odi,ti']; od(idx(k)+1:end,:)];
        end
    end
    if nargin>1  % interpolate with imut
        tsi = (imut(end,end)-imut(1,end))/length(imut);  % imu mean sampling interval
        szod = size(od,2);
        od1 = repmat(imut(:,end), 1, szod);
        for k=1:szod-1
            od1(:,k) = interp1(od(:,end), od(:,k), imut(:,end), 'nearest');
        end
        od1(:,1:end-1) = od1(:,1:end-1)*tsi/ts;
    end

