function [T, Dxyz] = Dblh2Dxyz(blh, Dblh)
% Convert perturbation error in geographic coordinate to ECEF
% Cartesian coordinate.
%
% Prototype: [T, Dxyz] = Dblh2Dxyz(blh, Dblh)
% Inputs: blh - geographic position [lat;lon;hgt]
%         Dblh - position perturbation [dlat;dlon;dhgt]
%                where dlat,dlon in radians, dhgt in meter
% Outputs: T - transformation matrix from nav-frame to Earth-frame, T=C^e_n*M_pv
%          Dxyz - position perturbation in Earth-frame in meters
%
% See also  blh2xyz, pos2cen.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/08/2013
global glv
    B = blh(1); L = blh(2); H = blh(3);
    sB = sin(B); cB = cos(B); sL = sin(L); cL = cos(L);
    N = glv.Re/sqrt(1-glv.e2*sB^2);
    NH = N+H;
    T = [ -NH*sB*cL,         -NH*cB*sL,  cB*cL;
          -NH*sB*sL,          NH*cB*cL,  cB*sL;
           (NH-glv.e2*N)*cB,  0,         sB ];
    if nargin==2
        Dxyz = T*Dblh;
    end