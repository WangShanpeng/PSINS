function a = adj3(A)
% The adjoint matrix of 3-order square matrix
% See also  det3, inv3, adj3, svd3, foam

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/01/2020
	a = [ (A(2,2)*A(3,3)-A(2,3)*A(3,2)), -(A(2,1)*A(3,3)-A(2,3)*A(3,1)),  (A(2,1)*A(3,2)-A(2,2)*A(3,1));
         -(A(1,2)*A(3,3)-A(1,3)*A(3,2)),  (A(1,1)*A(3,3)-A(1,3)*A(3,1)), -(A(1,1)*A(3,2)-A(1,2)*A(3,1));
          (A(1,2)*A(2,3)-A(1,3)*A(2,2)), -(A(1,1)*A(2,3)-A(1,3)*A(2,1)),  (A(1,1)*A(2,2)-A(1,2)*A(2,1)) ]';
