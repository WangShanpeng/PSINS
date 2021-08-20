function data = setclm(data, clm_idx, val)
% Set column data as specific value.
%
% Prototype: data = setclm(data, clm_idx, val)
% Inputs: data - data with repeated rows
%         clm_idx - column for index
%         val - value to be set
% Output: data - data with column clm_idx set by val
%
% See also  setrep0.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 11/03/2021
    if nargin<3, val=0; end
    [m,n] = size(val);
    if n==1, val=repmat(val,1,length(clm_idx)); end
    if m==1, val=repmat(val,size(data,1),1); end
    data(:,clm_idx) = val;
    