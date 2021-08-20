function [maxd, iter] = maxeig(M)
% Resolve max eigenvalue for symmetric matrix using iteration method.
% Example: 
%     M = randn(4)*diag(10.^[1:4]); [maxd, iter] = maxeig(M*M');
%
% See also  tr3, det3, inv3, adj3, svd3, foam, B33M44, svdest, vortech

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/01/2020
    d = diag(M);
    M1 = abs(M-diag(d));
    maxd = min(max(d+sum(M1,1)'), max(d+sum(M1,2)));
    db0 = 0; I = eye(length(d));
    for iter=1:20
        fM = maxd*I-M;
        db = det(fM)/trace(adj(fM));
        maxd = maxd - db;
        mk(iter) = maxd;
        if abs(db-db0)<1e-14, break; end
        db0 = db;
    end
    if exist('mk','var')
        figure, semilogy(abs(mk-max(eig(M)))); grid on
    end
    