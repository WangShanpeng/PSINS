function xy = gkprjBatch(BL, lgiCenter)
% Gauss-Kruger projection.
%
% Prototype: xy = gkprjBatch(BL, lgiCenter)
% Inputs: BL - [latitude; longitude] array in rad
%         lgiCenter - central meridian in rad
% Output: xy - xy coordinate
%
% See also  gkpinit, gkprj, gkpinv.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/12/2021
    if nargin<2, lgiCenter=0; end
    gk = gkpinit();
    xy = BL;
    for k=1:size(BL,1)
        xy(k,1:2) = gkprj(gk, BL(k,1:2)', lgiCenter)';
    end
