function [AzEl, BLH0, Cen] = topocent(xyz0, LOS)
% Transform line-of-sight vector(s) Ve into topocentric coordinate viewed
% at origin xyz0, where Ve and xyz0 are expressed in ECEF frame.
%
% Prototype: [AzEl, BLH0, Cen] = topocent(xyz0, Ve)
% Inputs: xyz0 - coordinate [X, Y, Z] in ECEF.
%         LOS - line-of-sight vector(s) viewed at origin xyz0
% Outputs: AzEl - azimuth(s) and elevation(s)
%          BLH0 - geographical coordinate(s) [latitude, longitude, altitude]
%          Cen - transformation matrix from e-frame to n-frame
%
% See also  xyz2blh, blh2xyz, pos2cen.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/08/2013
    if xyz0'*xyz0<1,  xyz0(2) = 1;  end  % to avoid collapse
    [BLH0,Cen] = xyz2blh(xyz0);
    Ln = LOS*Cen;  % Ln: LOS in receiver n-frame
    AzEl = [atan2(Ln(:,1), Ln(:,2)), atan(Ln(:,3)./sqrt(Ln(:,1).^2+Ln(:,2).^2))];
    idx = find(AzEl(:,1)<0);
    AzEl(idx,1) = AzEl(idx,1) + 2*pi;
    