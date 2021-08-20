function data = setrep0(data, clm_idx)
% Set repeated data as 0.
%
% Prototype: data = setrep0(data, clm_idx)
% Inputs: data - data with repeated rows
%         clm_idx - column for index
% Output: data - data with repeated rows set by 0
%
% See also  norep, delrepeat, imurepair, imuresample.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 17/06/2020
    if nargin<2, clm_idx=1:size(data,2); end
    dd = sum(abs(data(:,clm_idx)),2);
    row_idx = diff(dd)~=0;  row_idx = [1>0;row_idx];
    data(~row_idx,clm_idx) = 0;
    