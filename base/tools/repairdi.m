function data = repairdi(data, t0, t1)
% Repair data by differential & intergral method.
%
% Prototype: data = repairdi(data, t0, t1)
% Inputs: data - data input [data, t]
%         t0,t1 - time interval to repair
% Output: data - data output after repair
%
% See also  replaceby.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/01/2022
    if size(data,2)==1; data = [data, (1:length(data))']; end
    idx = data(:,end)>=t0 & data(:,end)<=t1;
    d = diff(data);
    d = [d, (1:length(d))'];  d1 = d;
    d(idx,:) = [];
    for k=1:size(d1,2)-1
        d1(:,k) = interp1(d(:,end), d(:,k), d1(:,end), 'linear');
    end
    data = cumsum([data(1,:); d1(:,1:end-1)],1);