function data = datahalf(data, n, N)
% 'Half' of the data (or n^th fraction of the data).
%
% Prototype: data = datahalf(data, n, N)
% Inputs: data - input data
%         n - n^th fraciton
%         N - total fraction
% Output: data - the n^th fration of the output data
%
% See also  datacut, datacuts.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/11/2022
    if nargin<3, N=2; end
    len = length(data);
    data = data(fix((n-1)/N*len)+1:fix(n/N*len),:);