function rng0(st)
% Set random seed.
%
% Prototype: rng0(st)
% Inputs: st - =0 for using previous random seed; =1 for new seed (default)
%
% See also  htwn, rng.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/05/2024
global rng_s
    if isempty(rng_s)
        rng_s = rng;
    end
    if st==0
        rng(rng_s);
    else
        rng_s = rng;
    end