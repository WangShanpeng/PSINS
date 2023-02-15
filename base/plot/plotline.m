function plotline(x, y)
% Plot horizontal lines.
%
% Prototype: plotline(x, y)
% Inputs: x, - x-axis/y-axis values
%
% See also  lbdef.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/01/2023
    len = length(y);
    y1 = zeros(2, len);
    for k=1:len
        y1(:,k) = [y(k);y(k)];
    end
    plot([x(1);x(end)], y1);