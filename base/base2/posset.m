function pos = posset(pos0, lon, hgt, isdeg)
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
    if nargin<4, isdeg=1; end 
    if nargin>=3,  pos0 = [pos0; lon; hgt];
    elseif nargin==2,  pos0 = [pos0; lon; 0];  end  % pos = posset(lat, lon)
    if length(pos0)==1, pos0 = [pos0;0;0]; end      % pos = posset(lat)
    if isdeg==1
        pos = [pos0(1:2)*glv.deg; pos0(3)];
    elseif isdeg==2
        pos = [dm2r(pos0(1)); dm2r(pos0(2)); pos0(3)];
    elseif isdeg==3
        pos = [dms2r(pos0(1)); dms2r(pos0(2)); pos0(3)];
    end