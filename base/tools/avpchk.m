function avp = avpchk(avp)
% AVP setting check.
%
% Prototype: avp = avpchk(avp)
% Input: avp - short form AVP input
% Output: avp - =[att;vn;pos]
%
% See also: avpset.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/01/2021
global glv
    err = 0;
    avp = avp(:);
    if length(avp)==1, avp=[0;0;avp(1); 0;0;0; glv.pos0];   % yaw
    elseif length(avp)==2, avp=[0;0;avp(1); 0;0;0; avp(2);0;0];  % yaw & lat
    elseif length(avp)==3, avp=[avp(1:3); 0;0;0; glv.pos0];  % att
    elseif length(avp)==4, avp=[avp(1:3); 0;0;0; avp(4);0;0];  % att & lat
    elseif length(avp)==6, avp=[avp(1:3); 0;0;0; avp(4:6)]; % att & pos
    elseif length(avp)==7, avp=[avp(1:3); a2mat(avp(1:3))*[0;avp(4);0]; avp(5:7)]; % att,vby & pos
    elseif length(avp)>=9, avp=avp(1:9);  % att,vn & pos
    else err=1; end
    if err==0
        absavp = abs(avp);
        if absavp(1)>pi/2 || absavp(2)>pi || absavp(3)>pi || norm(avp(4:6))>10*1000 ||...
                absavp(7)>pi/2 || absavp(8)>pi || absavp(9)>100*1000  % boundary check
            err = 1;
        end
    end
    if err==1, error('avp setting error.'); end
