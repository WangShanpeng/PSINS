function C01 = rxyz(ang, xyz)
% Rotation by x,y or z axis with angle 'ang'.
%
% Prototype: C01 = rxyz(ang, xyz)
% Inputs: ang - angle in rad
%         xyz - rotation axis 1/x/X, 2/y/Y or 3/z/Z
% Output: C01 - corresponding DCM = C^0_1
% 
% See also  rv2m, rv2q, a2mat, rotv.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 19/11/2021
    if nargin<2, xyz=3; end
	s = sin(ang); c = cos(ang);
    switch xyz
        case {1, 'x', 'X'},
            C01 = [1 0 0; 0 c -s; 0 s c];
        case {2, 'y', 'Y'}
            C01 = [c 0 s; 0 1 0; -s 0 c];
        case {3, 'z', 'Z'}
            C01 = [c -s 0; s c 0; 0 0 1];
    end
