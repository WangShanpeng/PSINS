function tscale = tscaleget()
% x-axis time scale get.
%
% Prototype: tscale = tscaleget()
% Output: tscale - time scale.
%
% See also  tscalepush, tscalepop, tscaleset, tscaletrans.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/07/2021
    global glv
    tscale = glv.tscale(end);
