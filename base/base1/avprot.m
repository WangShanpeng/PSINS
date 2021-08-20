function avp = avprot(avp, mu)
% AVP correction by install angles.
%
% Prototype: avp = avprot(avp, mu)
% Inputs: avp - input AVP
%         mu - install angles
% Output: avp - output AVP after install angle correction
%
% See also: aa2mu, aa2phimu, attrottt, imurot.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/01/2021
    if length(mu)==1, mu=[0,0,mu]; end
    Cbb = a2mat(mu)';
    for k=1:length(avp)
        avp(k,1:3) = m2att(a2mat(avp(k,1:3)')*Cbb)';
    end
    
    