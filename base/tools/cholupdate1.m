function R = cholupdate1(R, X, sgn)
% Rank 1 update to Cholesky factorization of R'*R+sgn*V*V'.
% Ref. https://www.cnpython.com/qa/168235,  http://en.wikipedia.org/wiki/Cholesky_decomposition
%
% Prototype: R = cholupdate1(R, X, sgn)
% Inputs: R - upper triangle input
%         X - column vector
%         sgn - 1/'+' for update, -1/'-' for downdate
% Output: R - upper triangle after update
%
% Example:
%   A = randn(4); A = A*A'; R = chol(A); X = randn(4,1)*0.1; 
%   R1 = cholupdate(R, X, '-'); R2 = cholupdate1(R, X, -1); err=R1-R2
%
% See also  cholupdate, chol1.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/11/2022
    if nargin<3, sgn=1; end
    if ischar(sgn), if sgn=='+', sgn=1; elseif sgn=='-'; sgn=-1; end; end
    n = length(X);
    X = X(:)';  % to row-vector
%     for k=1:n
%         r = sqrt(R(k,k)^2 + sgn*X(k)^2);
%         c = r/R(k,k);
%         s = X(k)/R(k,k);
%         R(k,k) = r;
%         R(k,k+1:n) = (R(k,k+1:n) + sgn*s*X(k+1:n))/c;
%         X(k+1:n) = c*X(k+1:n) - s*R(k,k+1:n);
%     end

    for k=1:n
        s11 = sqrt(R(k,k)^2+sgn*X(k)^2);
        c = R(k,k)/s11;  s = X(k)/s11;
        s12 = c*R(k,k+1:n) + sgn*s*X(k+1:n);
        X(k+1:n) = c*X(k+1:n) - s*R(k,k+1:n);
        R(k,k:n) = [s11,s12];
    end    