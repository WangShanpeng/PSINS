function gt = unixt2gpst(ut, leap)
% Trans UNIX time to GPS time.
%
% Prototype: gt = unixt2gpst(ut, leap)
% Inputs: ut - UNIX time (sec)
%         leap - leap second
% Output; gt - GPS time within a day
%
% See also  getat, tshift, adddt.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/02/2023
    if nargin<2, leap=18; end
    gt = mod(ut+leap-315964800, 86400);