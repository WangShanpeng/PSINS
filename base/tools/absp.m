function p = absp(data)
% To get absolute positive value.
%
% Prototype: p = absp(data)
% Inputs: data - data source input
%         p - positive output, if data>0 then p=data, else p=0
%
% See also  abs.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/04/2024
    p = zeros(size(data));
    idx = data>0;
    p(idx) = data(idx);