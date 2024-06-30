function xlimall(x0, x1, fign)
% In current figure (or fign), set all the X-axes limits to [x0, x1];
%
% Prototype: xlimall(x0, x1, fign)
% Inputs: x0, x1 - xlim value.
%         fign - figure NO.
%
% See also  ylimall, gridall, xlabelall, scft0, xygo, mylegend.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 04/08/2021
    if nargin<3, h=gcf; else, h=figure(fign); end
    if nargin<2,
        if length(x0)<2, x1=x0;      x0=0;
        else             x1=x0(2);   x0=x0(1);   end
    end
    ax = findall(h, 'type', 'axes');
    for k=1:length(ax)
        if isempty(get(ax(k),'Tag')), xlim(ax(k), [x0, x1]); end  % not for legend
    end
