function avperr = avperrset(phi, dvn, dpos)
% avp errors setting.
%
% Prototype: avperr = avperrset(phi, dvn, dpos)
% Inputs: phi - platform misalignment angles. all in arcmin
%         dvn - velocity errors in m/s
%         dpos - position errors dpos=[dlat;dlon;dhgt], all in m
% Output: avperr = [phi; dvn; dpos]
% 
% See also  poserrset, vperrset, avpadderr, imuerrset, avpset, insupdate, avperrstd.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/03/2014
global glv
    avperr = [rep3(phi)*glv.min; vperrset(dvn,dpos)];
