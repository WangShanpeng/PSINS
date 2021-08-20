function data = adddt(data, dt)
% Add the time tag data(:,end) with dt.
%
% Prototype: data = adddt(data, dt)
%
% See also  getat, sortt, tshift, delbias.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/11/2020
    if nargin<2, dt=-data(1,end); end
    data(:,end) = data(:,end)+dt;