function c = VmkXkn(a, b, K)
% Example
%   m=4; a=randn(m); b=randn(m);  VmkXkn(reshape(a',1,m*m),reshape(b',1,m*m))-reshape((a*b)', 1,m*m)

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/10/2023
    if nargin<3, 
        K = sqrt(size(a,2));
    end
    M = size(a,2)/K;  N = size(b,2)/K;
    c = zeros(size(a,1),M*N);
    for m=1:M
        for n=1:N
            m1 = N*(m-1)+n;
            for k=1:K
                c(:,m1) = c(:,m1) + a(:,M*(m-1)+k).*b(:,N*(k-1)+n);
            end
        end
    end
