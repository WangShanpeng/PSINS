function vperr = vperrset(dvn, dpos)
% avp errors setting.
%
% Prototype: vperr = vperrset(dvn, dpos)
% Inputs: dvn - velocity errors in m/s
%         dpos - position errors dpos=[dlat;dlon;dhgt], all in m
% Output: vperr = [dvn; dpos]
% 
% See also  avperrset, poserrset, avpadderr, imuerrset, avpset, insupdate.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/02/2021
    vperr = [rep3(dvn); poserrset(dpos)];
