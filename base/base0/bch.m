function [rv, rv0, rv1] = bch(rv1, rv2)
% Combination of two rotation vector (Baker-Campbell-Hausdorff Equation).
%
% Prototype: [rv, rv0] = bch(rv1, rv2)
% Input: rv1, rv2 - input rotation vector 1 & 2
% Output: rv - output ratation vector
%         rv0 - true ratation vector 
% 
% See also  askew, rv2m.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/01/2024
    mrv1 = askew(rv1); mrv2 = askew(rv2);
    rv = rv1+rv2 + 1/2*iaskew(mrv1*mrv2-mrv2*mrv1);
    rv0 = m2rv(rv2m(rv1)*rv2m(rv2)); % truth
%     rv1 = rv1+rv2 + iaskew(mrv1*mrv2); % the same as rv
    rv1 = rv1+rv2 + 2/3*cross(rv1,rv2); % the same as rv
