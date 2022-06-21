function dxyz = dpos2dxyz(dpos, pos0)
% Transfer Delta_[lat,lon,hgt] to [dx,dy,dz].
%
% Prototype: dxyz = dpos2dxyz(dpos, pos0)
% Inputs: dpos - [Delta_[lat, lon, hgt], t]
%         pos0 - see as original position
% Output: dxyz - [dx/East, dy/North, dz/Up] , Cartesian coordinates(E-N-U in meters) relative to pos0
%
% See also  pos2dxyz.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 11/01/2022
    if nargin<2, pos0 = zeros(3,1); end;
    n=4;
    if size(dpos,2)==3, dpos = [dpos, (1:length(dpos))']; n=3; end
    pos = [[pos0;0]'; [dpos(:,1)+pos0(1),dpos(:,2)+pos0(2),dpos(:,3)+pos0(3), dpos(:,4)]];
    dxyz = pos2dxyz(pos, pos0);
    dxyz = dxyz(2:end,:);
    if n==3;  dxyz=dxyz(:,1:3); end

