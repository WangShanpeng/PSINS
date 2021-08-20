function avp = avpset(att, vn, pos)
% avp=[attitude; velocity; position] setting.
%
% Prototype: avp = avpset(att, vn, pos)
% Inputs: att - attitude in deg
%        vn - velocity in m/s
%        pos - postion=[lat;lon;height] with lat and lon in deg,
%              while heigth in m. see posset.
% Output: avp=[attitude; velocity; position]
% 
% See also  posset, avpchk, avperrset, avpadderr, insupdate.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/03/2014, 22/01/2021
global glv
    if nargin==2, pos=vn; vn=zeros(3,1); end  % avp = avpset(att, pos);
    if length(att)==1,  att = [att; att; att];  end
    if length(vn)==1,   % vn is scalar, representing the forward velocity
        vb = [0; vn; 0];
        vn = a2mat(d2r(att(:)))*vb;
    end
    if length(pos)==1; pos=[pos;0;0]; end
    avp = avpchk([att(:)*glv.deg; vn(:); pos(1:2)*glv.deg; pos(3)]);
    