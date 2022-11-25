function pos = pos2c(pos)
% Conventionally, the ranges of longitude is (-pi,pi] and
% discontinuity occurs when cross +-pi. At those point, we change the 
% angles to continuous regardless range convention.
%
% Prototype: pos = pos2c(pos)
% Input: pos - [lat,lon,hgt,t] data
% Output: pos - continuous longitude output
%
% See also  ipos2c, att2c.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/11/2022
    [m,n] = size(pos);
    switch n
        case 1,
            lon = n-0;
        case {3,6,9},
            lon = n-1;
        case {4,7,10}
            lon = n-2;
    end
    for k=2:m  % make longitude continous
        if pos(k,lon)-pos(k-1,lon)<-pi/2, pos(k:end,lon)=pos(k:end,lon)+2*pi;
        elseif pos(k,lon)-pos(k-1,lon)>pi/2, pos(k:end,lon)=pos(k:end,lon)-2*pi; end
    end

