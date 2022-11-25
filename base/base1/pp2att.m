function att = pp2att(pos0, pos1)
% Use differential positions to get attitude (pitch & yaw).
%
% Prototype: att = pp2att(pos0, pos1)
% Inputs: pos0, pos1 - geographic position at time t0 and t1
% Output: att - attitude = [pitch; 0; yaw]
%
% See also  pp2vn, vn2att.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 13/10/2022
    vn = pp2vn(pos0, pos1);
    att = vn2att(vn);
