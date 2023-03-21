function CW = Polmat(xp, yp)
% Polar motion matrix.
%
% Prototype: CW = Polmat(xp, yp)
% Inputs: xp,yp - polar motion in rad
% Output: CW - polar motion matirx (C^e_E)
%
% See also  Precmat, Nutmat.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/12/2021
	xyp = xp-yp;
	CW = [ 1-xp*xp/2,	0.0,			xp;
		   0.0,         1-yp*yp/2,		-yp;
		  -xp,          yp,				1-xyp*xyp/2 ];