function qo = qinv(qi)
% Quaternion inverse.
% 
% Prototype: qo = qinv(qi)
% Input: qi - input quaternion
% Output: qo - output quaternion, qo = qi^-1;
% 
% See also  qconj, qnormlz, qeye, qmul, qmulv.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/11/2007
    qo = [qi(1); -qi(2:4)]/(qi'*qi);