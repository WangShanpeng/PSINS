function [data,dt] = adddt(data, dt)
% Add the time tag data(:,end) with dt.
%
% Prototype: [data,dt] = adddt(data, dt)
% Inputs: data - data input
%         dt - time to shift
% Output; data - data output
%         dt - time to shift
%
% See also  getat, sortt, tshift, delbias, scalet, addclmt.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/11/2020
    if nargin<2, dt=-data(1,end); end
    if dt==0, return; end
    data(:,end) = data(:,end)+dt;