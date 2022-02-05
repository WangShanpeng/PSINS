function q = q42q3(q)
% Translate q = [q0 q1 q2 q3] to q = [q1 q2 q3], for |q|=1 and q0>=0 can be omitted.
%
% Prototype: q = q42q3(q)
% Input: q - = [q0 q1 q2 q3]
% Output: q - = [q1 q2 q3], default for q0>=0
%
% See also  q32q4

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/11/2021
    if size(q,2)==1
        if q(1)>0, q = q(2:4);
        else, q = -q(2:4); end
    else
        idx = q(:,1)<0;
        q(idx,2:4) = -q(idx,2:4);
        q = q(:,2:end);
    end