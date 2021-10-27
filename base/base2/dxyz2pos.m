function pos = dxyz2pos(dxyz, pos0)
% Transfer [lat,lon,hgt] to [dx,dy,dz].
%
% Prototype: pos = dxyz2pos(dxyz, pos0)
% Inputs: dxyz - [dx/East, dy/North, dz/Up] , Cartesian coordinates(E-N-U in meters) relative to pos0
%         pos0 - initial position
% Output: pos - [lat, lon, hgt, t]
%
% Example:
%   x = (0:100:100000)'; dxyz=[x,x,x];
%   pos = dxyz2pos(dxyz);  dxyz1 = pos2dxyz(pos);
%   figure, plot(dxyz1-dxyz);
%
% See also  pos2dxyz, dratt, pos2dplot.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/01/2021
global glv
    if nargin<2, pos0 = glv.pos0; end;
    ddxyz = diff([[0,0,0];dxyz(:,1:3)],1);
    pos0 = repmat(pos0',size(ddxyz,1),1);
    pos = pos0;
    for k=1:3  % iteration for calcuating DR pos, faster than element for-loop 
        [RMh, clRNh] = RMRN(pos);
        dpos = [ddxyz(:,2)./RMh, ddxyz(:,1)./clRNh, ddxyz(:,3)];
        dpos = cumsum(dpos,1);
        pos = pos0 + dpos;
    end
    if size(dxyz,2)>3,  pos(:,4) = dxyz(:,4); end
