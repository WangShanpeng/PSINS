function A1 = inv3(A)
% The inverse of 3-order square matrix
% See also  tr3, det3, inv3, adj3, svd3, foam

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/01/2020
    A1 = adj3(A)/det3(A);
