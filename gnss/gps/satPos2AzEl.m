function [AzEl, BLH] = satPos2AzEl(satPos, xyz0)
% Calculate satellite azimuth(s) and elevation(s), not taking into
% account the Earth rotation.
%
% Prototype: [AzEl, BLH] = satPos2AzEl(satPos, xyz0)
% Inputs: satPos - satellites positions in ECEF
%         xyz0 - receiver position in ECEF, or pos=[lat;lon;hgt];
% Outputs: AzEl - satellite azimuth(s) and elevation(s)
%          BLH - receiver position represented by [lat lon hgt]
%
% See also  rhoSatRec. topocent.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 31/07/2015
    if norm(xyz0(1:2))<10,  BLH=xyz0; Cen=pos2cen(BLH); xyz0=blh2xyz(BLH);  % lat-lon-hgt format
    else           BLH=xyz2blh(xyz0); Cen = pos2cen(BLH);  end              % x-y-z ECEF format
    dpos = satPos(:,1:3) - repmat(xyz0',length(satPos),1);
    rho = sqrt(dpos(:,1).^2+dpos(:,2).^2+dpos(:,3).^2);
    LOS = dpos./[rho,rho,rho];
    % topocent
    Ln = LOS*Cen;  % Ln: LOS in receiver n-frame
    AzEl = [atan2(Ln(:,1), Ln(:,2)), atan(Ln(:,3)./sqrt(Ln(:,1).^2+Ln(:,2).^2))];
    idx = find(AzEl(:,1)<0);
    AzEl(idx,1) = AzEl(idx,1) + 2*pi;
