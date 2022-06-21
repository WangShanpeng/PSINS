function tscale = tscalepop()
% x-axis time scale pop.
%
% Prototype: tscale = tscalepop()
% Output: tscale - time scale.
%
% See also  tscalepush, tscaleget, tscaleset.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/07/2021
    global glv
    tscale = glv.tscale(end);
    if length(tscale)>1
        glv.tscale = glv.tscale(1:end-1,:);
    else
        glv.tscale = 1;
    end