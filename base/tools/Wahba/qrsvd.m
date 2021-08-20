function [u, s, v, C] = qrsvd(A)
% Singular value decomposition using qr method 
%
% Prototype: [u, s, v, C] = qrsvd(A)
% Input:   A     - matrix
% Outputs: u,s,v - u and v are unitary matrices, s is a diagonal matrix,
%                  so that A = u*s*v'.
% Examples
%    A=randn(3); [u, s, v, C] = qrsvd(A*diag(100.^[1:3])); C*C'
%
% See also  tr3, det3, inv3, adj3, svd3, foam, B33M44, svdest, vortech, maxeig, qrmgs

% Copyright(c) 2009-2016, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/02/20
    s = A'; ds0 = diag(s);
    u = eye(3); v = u;
    for k=1:3
        [uk, s] = qrmgs(s'); u = u*uk;
        [vk, s] = qrmgs(s'); v = v*vk;
        sk(k,:) = diag(s)';
        ds = diag(s);
        if max((ds-ds0)./ds)<1e-3, break; end
        ds0 = ds;
    end
    if nargout==4
        C = u*diag(sign(diag(s)))*v';
    end
    if exist('sk','var')
        figure, plot(sk); grid on
    end
        

