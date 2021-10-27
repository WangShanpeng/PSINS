function data = abnomaladd(data, ratio, sigma, mu)
% Add some abnomal value to the data.
%
% Prototype: data = abnomaladd(data, ratio, sigma, mu)
% Inputs: data - data input
%         ratio - abnomal ratio = 0~1
%         sigma,mu - abnomal value is normal distribution with std=sigma,
%         mean=mu.
% Output: data - data output with abnomal
%
% Example:
%   data = abnomaladd(randn(100,2), 0.2, 100);  myfig; plot(data); xygo
%
% See also smoothol.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/10/2021
    if nargin<4, mu=0; end
    [len, n] = size(data);
    r = fix(len*ratio);
    if r<1, r=1; elseif r>len/2, r=fix(len/2); end
    idx = fix(rand(r, n)*len);  ni = size(idx,1);
    for k=1:n
        idx(idx(:,k)==0,k)=1;
        data(idx(:,k),k) = randn(ni,1)*sigma + mu;
    end