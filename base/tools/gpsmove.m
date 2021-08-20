function  gps1 = gpsmove(gps, pos0)
% Move gps to a specific place, whose first point is at pos0.
%
% See also  apmove, gpsplot.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 14/10/2020
    n = size(gps,2);
    gps1 = gps;
    if n>6
        nL = 4;
    else
        nL = 1;
    end
    dlat = gps(:,nL)-gps(1,nL);
	gps1(:,nL) = pos0(1)+dlat;
    dlon = gps(:,nL+1)-gps(1,nL+1);
	gps1(:,nL+1) = pos0(2)+dlon.*cos(gps(:,nL))./cos(gps1(:,nL)); 
