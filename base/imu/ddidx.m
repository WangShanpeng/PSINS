function dd = ddidx(dd, idx, ddunit, ts)
% Extract sub-data from a data array.
%
% Prototype: dd = ddidx(dd, idx, ddunit, ts)
% Inputs: dd - data contains sub-dd
%         idx - column index of sub-dd
%         ddunit - data unit
%         ts - sampling interval
% Output: dd - sub-dd
%
% See also: imuidx, gpsidx, avpidx.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/11/2022
    dd = dd(:,abs(idx));
    n = length(idx);
    if exist('ts','var'), dd(:,n)=dd(:,n)*ts; n=n-1; end
    if ~exist('ddunit','var'), ddunit=1; end
    if ddunit~=1, dd(:,1:n)=dd(:,1:n)*ddunit; end
    for k=1:n
        if (idx(k))<0, dd(:,k)=-dd(:,k); end
    end
    