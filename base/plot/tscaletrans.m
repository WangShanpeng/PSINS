function data = tscaletrans(data, type)
% x-axis time scale push.
%
% Prototype: data = tscaletrans(data)
% Inputs: data - data with the last column as time tag.
%         type - time type
% Output: data - whose time tag is changed
%
% See also  tscalepush, tscalepop, tscaleget.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/07/2021
    if nargin<2
        data(:,end) = data(:,end)/tscaleget();
    else
        tscalepush(type);
        data(:,end) = data(:,end)/tscaleget();
        tscalepop();
    end