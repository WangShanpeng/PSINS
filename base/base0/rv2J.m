function [J, J1] = rv2J(rv)
% Calculate the Jacobian matrix of rotation vector.
%
% Prototype: J = rv2J(rv)
% Input: rv - rotation vector
% Outputs: J - Jacobian matrix
%          J1 - inverse of J
% 
% See also  rv2m, rv2q.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/05/2023
	nrv = sqrt(rv'*rv);
    if nrv>1e-6, 
        u=rv/nrv; snn=sin(nrv)/nrv;
        J = snn*eye(3)+(1-snn)*u*u'+(1-cos(nrv))/nrv*askew(u);
    else
        J = eye(3);
    end
    if nargout==2, J1=J^-1; end


