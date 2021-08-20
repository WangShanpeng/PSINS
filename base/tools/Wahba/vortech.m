function [m, q, iter] = vortech(B, tel)
% Vectorial ORthogonalization TECHnique (VORTECH)
% Example: [m, q, iter] = vortech(randn(3));
% Refs. BAR-ITZHACK, Orthogonalization Techniques of a Direction Cosine Matrix, 1969.
%       WU, A Simple Method Free of SVD and Eigen-Decomposition, 2018
%
% See also  tr3, det3, inv3, adj3, svd3, foam, B33M44, svdest, quest

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/01/2020
    % assert(det(B)>0)
    if nargin<2, tel = 1e-12; end
    n20 = 0;
    for iter=1:50
        n2 = trace(B*B');
        B = (B+adj3(B')) / ((n2+1)/2);
        if abs(n2-n20)<tel,
            break;
        end
        n20 = n2;
    end
    m = B;
    if nargout>=2, q = m2qua(m); end
    