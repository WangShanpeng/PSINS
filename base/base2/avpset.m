function avp = avpset(att, vn, pos, isdeg)
% avp=[attitude; velocity; position] setting.
%
% Prototype: avp = avpset(att, vn, pos)
% Inputs: att - attitude in deg
%        vn - velocity in m/s
%        pos - postion=[lat;lon;height] with lat and lon in deg,
%              while heigth in m. see posset.
%        isdeg - unit deg flag
% Output: avp=[attitude; velocity; position]
% 
% See also  posset, avpchk, avperrset, avpadderr, insupdate.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/03/2014, 22/01/2021
global glv
    if nargin<4, isdeg=1; end 
    if nargin==2, pos=vn; vn=zeros(3,1); end  % avp = avpset(att, pos);
    if length(att)==1,  att = [att; att; att];  end
    if length(vn)==1,   % vn is scalar, representing the forward velocity
        vb = [0; vn; 0];
        vn = a2mat(d2r(att(:)))*vb;
    end
    if length(pos)==1; pos=[pos;0;0]; end
    if isdeg==1
        avp = avpchk([att(:)*glv.deg; vn(:); pos(1:2)*glv.deg; pos(3)]);
    elseif isdeg==2
        avp = avpchk([dm2r(att(1));dm2r(att(2));dm2r(att(3)); vn(:); dm2r(pos(1));dm2r(pos(2)); pos(3)]);
    elseif isdeg==3
        avp = avpchk([dms2r(att(1));dms2r(att(2));dms2r(att(3)); vn(:); dms2r(pos(1));dms2r(pos(2)); pos(3)]);
    else
        avp = avpchk([att(:); vn(:); pos(1:2); pos(3)]);
    end
    