function om = dv2om(v1, v2, k)
% Using double vectors to establish unit orthogonal matrix.
%
% Prototype: om = dv2om(v1, v2)
% Inputs: v1,v2 - two reference vectors
% Outputs: om - unit orthogonal matrix
%
% See also  dv2atti.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/07/2024
    vtmp1 = cross(v1,v2); vtmp2 = cross(vtmp1,v1);
    om = [v1/norm(v1), vtmp1/norm(vtmp1), vtmp2/norm(vtmp2)];
    if nargin<3, return; end
    if k==1
        om = [v1, v2, vtmp1];
    elseif k==2
        om = [v1/norm(v1), v2/norm(v2), vtmp1/norm(vtmp1)];
    end
