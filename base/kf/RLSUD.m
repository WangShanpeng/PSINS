function [U, D, K, X] = RLSUD(U, D, H, R, Z, X)
% Recursive Least Square filter using UD decomposition.
%
% Prototype: [U, D, K, X] = RLSUD(U, D, H, R, Z, X)
% Inputs: U,D - factors of UD decomposition
%         H,R - 1-row measurement matrix / measurement var
%         X,Z - state / measurement
% Outputs: U,D,X - as input after update
%          K - gain
%
% See also  KFUD, RLS, kfupdate.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/01/2023
    if isstruct(U)  % kf = RLSUD(kf, z);
        [U.Uk, U.Dk, ~, U.xk] = RLSUD(U.Uk, U.Dk, U.Hk, U.Rk, D, U.xk);
        return;
    end
    if nargin<4, R=1; end
    n = length(D);
    f = (H*U)';  g = D.*f;  afa = f'*g+R;
    for j=n:-1:1
        afa0 = afa - f(j)*g(j); lambda = -f(j)/afa0;
        D(j) = afa0/afa*D(j);   afa = afa0;
        for i=(j-1):-1:1
            s = (i+1):(j-1);
            U(i,j) = U(i,j) + lambda*(g(i)+U(i,s)*g(s));
        end
    end
    if nargout>2
        K = U*(D.*(H*U)')/R;
        if nargout>3, X = X + K*(Z-H*X); end
    end
    