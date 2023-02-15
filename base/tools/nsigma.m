function p = nsigma(n, inv)
% n-sigma probability of normal distribution, or, contrariwise.
%
% Prototype: p = nsigma(n)
% Inputs: n - n-sigma with [0,inf)
%         inv - inverse calculation flag
% Output: p - n-sigma probability
%
% Example
%   nsigma(nsigma(n),-1) == n
%
% See also  medianp, meann, maxn.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/01/2023
    if nargin<2, inv=1; end
    if inv==-1                 % n = nsigma(p, -1);   p>=0
        p = norminv((1+abs(n))/2);
        return;
    end
    p = 2*normcdf(abs(n))-1;  % n>=0