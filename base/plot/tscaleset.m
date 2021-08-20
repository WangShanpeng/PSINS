function tscaleset(val)
% x-axis time scale set.
%
% Prototype: tscaleset()
%
% See also  tscalepush, tscalepop, tscaleget.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/07/2021
    global glv
    if nargin<1, val=1; end
    glv.tscale = val;