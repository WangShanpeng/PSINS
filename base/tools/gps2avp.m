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
    if size(gps,2)==4  % gps=[pos,t]
        vn = pp2vn(gps);
        avp = gps2avp([vn(:,1:3), gps]);
        if isfig, insplot(avp); end
    elseif size(gps,2)==7  % gps=[vn,pos,t]
        att = vn2att(gps(:,[1:3,end]),0.3);
        avp = [att(:,1:3), gps];
        if isfig, insplot(avp); end
    end