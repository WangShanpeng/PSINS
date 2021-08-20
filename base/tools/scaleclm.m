function data = scaleclm(data, scale)
% Scale data for each column.
%
% Prototype: data = scaleclm(data, scale)
% Inputs: data - data to be scaled
%         scale - scale factor for each column
% Output: data - scaled data
%
% See also  normv, no0, norep.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 05/04/2021
    n = size(data,2);
    if length(scale)==1, scale=repmat(scale,1,n); end
    for k=1:n, data(:,k) = data(:,k)*scale(k); end
    