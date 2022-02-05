function data = scalet(data, s)
% Scale the time tag data(:,end) with s.
%
% Prototype: data = scalet(data, s)
%
% See also  getat, sortt, tshift, delbias, adddt.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/12/2021
    t0 = data(1,end);
    data(:,end) = (data(:,end)-t0)*s + t0;