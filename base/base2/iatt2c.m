function att = iatt2c(attc)
% The reverse of 'att2c'.
%
% Prototype: att = iatt2c(attc)
% Input: attc - input continuous Euler angle array
% Output: att - common Euler angle
%
% See also  att2c, angle2c, a2mat, m2att.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/12/2020
    if size(attc,2)>3   % avp = iatt2c(avp);
        att = [iatt2c(attc(:,1:3)),attc(:,4:end)];
        return;
    end
    s = sin(attc); c = cos(attc);
    si = s(:,1); sj = s(:,2); sk = s(:,3); 
    ci = c(:,1); cj = c(:,2); ck = c(:,3);
    C12 = -ci.*sk;
    C22 =  ci.*ck;
    C31 = -ci.*sj; C32 = si; C33 = ci.*cj;
    att = [ asin(C32), atan2(-C31,C33), atan2(-C12,C22) ];

