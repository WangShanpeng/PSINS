function avp = gps2avp(gps, isfig)
% trans GNSS array to AVP, where att is tracking attitude.
%
% Prototype: avp = gps2avp(gps)
% Input: gps - GNSS vel/pos array
% Output: avp - tracking attitude, vn and pos
%
% See also  vn2att, pp2vn.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 25/06/2021
    if nargin<2, isfig=0; end
    n = size(gps,2);
    if n==4 || n==5 % gps=[pos,t]
        vn = pp2vn(gps);
        avp = gps2avp([vn(:,1:3), gps(:,[1:3,end])]);
        if isfig, insplot(avp); subplot(321); title('GNSS pos to AVP'); end
    elseif n==7 || n==8  % gps=[vn,pos,t]
        att = vn2att(gps(:,[1:3,end]),0.3);
        avp = [att(:,1:3), gps(:,[1:6,end])];
        if isfig, insplot(avp); subplot(321); title('GNSS vn-pos to AVP'); end
    end