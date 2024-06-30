function xlabelall(lab, fign)
% In current figure (or fign), set all the X-label to lab];
%
% Prototype: xlimall(x0, x1, fign)
% Inputs: lab - X-label
%         fign - figure NO.
%
% See also  xlimall, gridall, scft0, xygo, mylegend.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/02/2024
    if nargin<2, h=gcf; else, h=figure(fign); end
    ax = findall(h, 'type', 'axes');
    for k=1:length(ax)
        if isempty(get(ax(k),'Tag')), xlabel(ax(k), lab); end  % not for legend
    end
