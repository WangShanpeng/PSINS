function att = pry(pitch, roll, yaw)
% Attitude = [pitch; roll; yaw] setting.
%
% Prototype: att = pry(pitch, roll, yaw)
% Input: att=[pitch; roll; yaw], in arcdeg,
% Output: att=[pitch*arcdeg; roll*arcdeg; yaw*arcdet]
% 
% See also  posset, llh, avpset.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/08/2024
global glv
    if nargin<2, yaw=pitch; pitch=0; roll=0;  end;   % att=pry(yaw);
    att = [pitch*glv.deg; roll*glv.deg; yaw*glv.deg];