function [data, idx] = monoinc(data)
% Make data to strictly monotonic increasing.
%
% Prototype:  [data, idx] = monoinc(data)
% Input: data - input pos data, the last column is time tag
% Outputs: data - output data
%          idx - non monotonic increasing data index
%
% See also  reminct, delbias, posbias.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 14/08/2024
    [t,idx] = sort(data(:,end));
    data = data(idx,:);
    dt = diff(data(:,end));
    idx = find(dt<=0)+1;
    data(idx,:) = [];