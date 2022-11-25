function pos = ipos2c(pos)
% The inverse of pos2c.
%
% Prototype: pos = ipos2c(pos)
% Input: pos - [lat,lon,hgt,t],etc. data
% Output: pos - longitude output within (-pi,pi]
%
% See also  ipos2c, att2c.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/11/2022
    [~,n] = size(pos);
    switch n
        case 1,
            lon = n-0;
        case {3,6,9},
            lon = n-1;
        case {4,7,10}
            lon = n-2;
    end
    pos(:,lon) = atan2(sin(pos(:,lon)),cos(pos(:,lon)));

