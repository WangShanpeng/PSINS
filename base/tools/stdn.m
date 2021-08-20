function [err, err1] = stdn(data)
% Statistics for some test error records.
%
% Prototype: res = meann(scr, n, dim)
% Inputs: data - cell for test error data 
%         err - error statistics
% Example:
%   data=[];
%   for k=1:10, data{k}=k+randn(10,2); end
%   [err, err1] = mnstd(data)
%
% See also  meann, sumn, maxn, avar.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 31/03/2021
    if nargin>1, data=varargin; end
    data1 = [];
    for k=1:length(data)
        merr(k,:) = mean(data{k});
        serr(k,:) = std(data{k});
        data1 = [data1; data{k}];
    end
    err = sqrt(std(merr).^2+mean(serr).^2);
    err1 = std(data1);
    