function poserr = poserrset(dpos0, dlon, dhgt)
% position errors dpos=[dlat;dlon;dhgt] setting.
%
% Prototype: poserr = poserrset(dpos0)
% Input: dpos0=[dlat; dlon; dhgt], NOTE: dlat, dlon and dhgt are all in m.
% Output: poserr=[dpos0(1)/Re; dpos0(2)/Re; dpos0(3)], so poserr(1)=dlat,
%              poserr(2)=dlon are in rad and poserr(3)=dhgt is in m.
% 
% See also  avperrset, vperrset, posset.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/03/2014
global glv
    if nargin==3, dpos0=[dpos0;dlon; dhgt]; end  % dpos = poserrset(dlat, dlon, dhgt)
    if nargin==2, dpos0=[dpos0;dpos0; dlon]; end  % dpos = poserrset(dlat_dlon, dhgt)
    dpos0 = rep3(dpos0);
    poserr = [dpos0(1:2)/glv.Re; dpos0(3)];
