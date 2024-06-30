function x = ar1filt(x, N)
% AR(1) filter.
%
% Prototype: x = ar1flt(x, N)
% Inputs: x - data array
%         N - AR(1) filter parameter = tau/ts
% Outputs:x data after filter
%
% See also  ar1coefs, markov1.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/01/2021
    if size(x,2)>1
        for k=1:size(x,2),  x(:,k) = ar1filt(x(:,k), N); end
        return;
    end
	[b, a] = ar1coefs(1, N);
	x = filtfilt(b, a, x);
%     x = filter(b, a, x);
