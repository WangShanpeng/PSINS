function [data, idx] = datadel(data0, t1, t2)
% delete data between time tags t1 & t2.
%
% Prototype: [data, idx] = datadel(data0, t1, t2)
% Inputs: data0 - input data, whose last column should be time index
%         t1, t2 - start & end time tags to delete
% Outputs: data, idxi - output data & index in data0
%
% See also  datacut, getat, combinet.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/10/2020
    if nargin<3, t2=data0(end,end); end
    i1 = find(data0(:,end)>=t1, 1, 'first');
    i2 = find(data0(:,end)<=t2, 1, 'last');
    idx = [1:i1,i2:length(data0)]';
    data = data0(idx,:);
