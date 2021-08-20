function pos = posset(pos0, lon, hgt)
% Geographic position = [latitude; logititude; height] setting.
%
% Prototype: pos = posset(pos0, lon, hgt, isdeg)
% Input: pos0=[lat; lon; height], where lat and lon are always in arcdeg,
%             & height is in m.
%           or pos0=[lat; lon; hgt].
% Output: pos=[pos0(1)*arcdeg; pos0*arcdeg; dpos0(3)]
% 
% See also  avpset, poserrset.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/03/2014
global glv
    if nargin==3,  pos0 = [pos0; lon; hgt];
    elseif nargin==2,  pos0 = [pos0; lon; 0];  end
    if length(pos0)==1, pos0 = [pos0;0;0]; end
    pos = [pos0(1:2)*glv.deg; pos0(3)]; 