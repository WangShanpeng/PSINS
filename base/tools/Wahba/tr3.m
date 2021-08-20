function t = tr3(A)
% The trace of 3-order square matrix
% See also  tr3, det3, inv3, adj3, svd3, foam

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/01/2020
    t = A(1,1) + A(2,2) + A(3,3);
    
%     t = trace(A);
