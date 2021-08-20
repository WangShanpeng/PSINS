function dxyz = pos2dxyz(pos, pos0)
% Transfer [lat,lon,hgt] to [dx,dy,dz].
%
% Prototype: dxyz = pos2dxyz(pos, pos0)
% Inputs: pos - [lat, lon, hgt, t]
%         pos0 - see as original position
% Output: dxyz - [dx/East, dy/North, dz/Up] , Cartesian coordinates(E-N-U in meters) relative to pos0
%
% See also  dxyz2pos, RMRN, pos2dplot, pos3vplot.

% Copyright(c) 2009-2017, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 19/06/2017, 26/01/2021
    if nargin<2, pos0 = pos(1,1:3)'; end;
    dpos = diff([pos0';pos(:,1:3)]);
    [RMh, clRNh] = RMRN(pos(:,1:3));
    dxyz = [dpos(:,2).*clRNh, dpos(:,1).*RMh, dpos(:,3)];
    dxyz = cumsum(dxyz);
    if size(pos,2)==4, dxyz(:,4) = pos(:,4); end

