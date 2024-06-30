function g = grav(pos)
% Normal gravity amplitude calculation.
%
% Prototype: g = grav(pos)
% Input: pos - =[lat, lon, hgt]
% Output: h - normal gravity
%
% See also  earth, egmwgs84, egm.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 20/05/2023
global glv
    if length(pos)<3, pos(3)=0; end
	sl2 = sin(pos(1))^2;  sl4 = sl2^2;
    g = glv.g0*(1+5.2790414e-3*eth.sl2+2.32718e-5*sl4)-3.086e-6*pos(3);  % GJB6304-2008,Eq.(B.5)
