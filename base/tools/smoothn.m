function data = smoothn(data, n)
% Smooth n-column data.
%
% Prototype: data = smoothn(data, n)
% Inputs: data - data in
%         n - number of points used to smooth
% Output: data - data out
%
% See also smoothol, interp1n.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/03/2022
    if nargin<2, n=5; end
    if n<=1; return; end
    for k=1:size(data,2)
        data(:,k) = smooth(data(:,k),n);
    end