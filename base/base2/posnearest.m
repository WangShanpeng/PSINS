function [idx, lidx, hidx] = posnearest(pos, pos0)
% Display latitude/longitude in degree, ang height in meter.
%
% Prototype: [idx, lidx, hidx] = posnearest(pos, pos0)
% Inputs: pos - [lat,lon,hgt] position array.
%         pos0 - one position point
% Outputs: idx - the nearest position index in pos array
%          lidx - the nearest leveling position index in pos array
%          hidx - the nearest height index in pos array
%          
% See also  pos2dxyz.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/08/2024
    dxyz = pos2dxyz(pos, pos0);
    xy2 = dxyz(:,1).^2 + dxyz(:,2).^2;
    z2  = dxyz(:,3).^2;
    xyz2 = xy2+z2;
    [~, idx] = min(xyz2);
    if nargout>1
        [~, lidx] = min(xy2);  [~, hidx] = min(z2);
    end
    
