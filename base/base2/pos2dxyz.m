function [dxyz,ddxyz,od] = pos2dxyz(pos, pos0)
% Transfer [lat,lon,hgt] to [dx,dy,dz].
%
% Prototype: [dxyz,ddxyz,od] = pos2dxyz(pos, pos0)
% Inputs: pos - [lat, lon, hgt, t]
%         pos0 - see as original position
% Outputs: dxyz - [dx/East, dy/North, dz/Up] , Cartesian coordinates(E-N-U in meters) relative to pos0
%          ddxyz - xyz increment
%          od - ODmeter distance increment
%
% See also  dxyz2pos, RMRN, pos2dplot, pos3vplot, dpos2dxyz, odsimu.

% Copyright(c) 2009-2017, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 19/06/2017, 26/01/2021
    if nargin<2, pos0 = pos(1,1:3)'; end;
    dpos = diff([pos0';pos(:,1:3)]);
    [RMh, clRNh] = RMRN(pos(:,1:3));
    ddxyz = [dpos(:,2).*clRNh, dpos(:,1).*RMh, dpos(:,3)];
    dxyz = cumsum(ddxyz,1);
    od = normv(ddxyz);
    if size(pos,2)==4, dxyz(:,4) = pos(:,4); od = [od,pos(:,4)]; end  % add time tag

