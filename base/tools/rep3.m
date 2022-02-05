function v31 = rep3(vxx)
% Repeat data to 3x1 vector.
%
% Prototype: v31 = rep3(vxx)
% Input: vxx - 1x1 or 2x1 vector input
% Output: v31 - 3x1 vector output
%
% See also  norep, no0, avperrset, poserrset.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/02/2021
    v31 = vxx(:);
    len = length(v31);
    if len>3; return; end
    if len==1,      v31 = [v31;v31;v31];
    elseif len==2   v31 = [v31(1);v31(1);v31(2)];  end
    v31 = v31(1:3);

