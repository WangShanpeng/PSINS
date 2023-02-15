function gps = gpsidx(data, idx, isdeg, isnorep)
% Extract GPS([vn,pos,t]) from a data array.
%
% Prototype: gps = gpsidx(data, idx, isdeg, isnorep)
% Inputs: data - data contains [vn,pos,t] or [pos,t] 
%         idx - column index of [vn,pos,t]
%         isdeg - [lat,lon] is in degree
%         isnorep - remove repeated data
% Output: gps - =[vn,pos,t] or [pos,t] 
%
% See also: imuidx, avpidx, ddidx, gpsplot.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 31/01/2021
global glv
    gps = data(:,abs(idx));
    if nargin<4, isnorep=1; end
    if nargin<3, isdeg=1; end
    if isdeg==1, gps(:,end-3:end-2)=gps(:,end-3:end-2)*glv.deg; end
    if isnorep==1, gps=no0(norep(gps,1),1:3); end
    for k=1:length(idx)
        if(idx(k))<0, gps(:,k)=-gps(:,k); end
    end
    