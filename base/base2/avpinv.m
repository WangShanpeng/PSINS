function avp = avpinv(avp)
% Reverse avp(t), with vn & eb negative.
%
% Prototype: avp = avpinv(avp)
% Input: avp - navigation results, avp = [att,vn,pos,t]
% Output: avp - navigation results, avp = [att,vn,pos,t]
%
% See also  insinstant, attpure.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/09/2023
    avp = flipud(avp);
    [m,n] = size(avp);
    idx = 4:6;
    if n>10, idx = [4:6,10:12]; end
    avp(:,idx) = - avp(:,idx);