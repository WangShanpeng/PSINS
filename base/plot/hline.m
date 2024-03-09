function hline(t, y)
% Plot horizontal lines.
%
% Prototype: hline(t, y)
% Inputs: t - x axis, at least two points; or from current figure
%         y - y axis
%
% Example
%    myfig, hline([1;10], randn(3,1));
% 
% See also  plotline.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/11/2023
    if nargin<2, y=t; t=get(gca,'xlim'); end;  % hline(y)
    t = [t(1);t(end)];
    hold on;
    for k=1:length(y)
        plot(t, [y(k); y(k)], '-.');
    end