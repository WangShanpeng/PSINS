function data = negclm(data, clm)
% Negative the specific data column, i.e. data(:,clm) = -data(:,clm).
%
% Prototype: data = negclm(data, clm)
% Inputs: data - input data
%         clm - data column to be set negative
% Output:
%         data = output data
%
% See also  datacut, datadel, combinedata.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/10/2021
    clmmax = size(data,2);
    if nargin<2
        clm = 1:(clmmax-1);
    end
    for k=1:length(clm)  % data(:,clm) = -data(:,clm);
        if clm(k)>clmmax, break; end
        data(:,clm(k)) = -data(:,clm(k));
    end