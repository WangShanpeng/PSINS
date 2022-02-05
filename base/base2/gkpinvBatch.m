function BL = gkpinvBatch(xy, lgiCenter)
% Gauss-Kruger projection.
%
% Prototype: BL = gkpinvBatch(xy, lgiCenter)
% Inputs: xy - xy coordinate array
%         lgiCenter - central meridian in rad
% Output: BL - [latitude; longitude] in rad
%
% See also  gkpinit, gkprj, gkpinv, gkprjBatch.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/12/2021
    if nargin<2, lgiCenter=0; end
    gk = gkpinit();
    BL = xy;
    for k=1:size(xy,1)
        BL(k,1:2) = gkpinv(gk, xy(k,1:2)', lgiCenter)';
    end
