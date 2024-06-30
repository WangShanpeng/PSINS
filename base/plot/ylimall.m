function ylimall(y0, y1, fign)
% In current figure (or fign), set all the Y-axes limits to [y0, y1];
%
% Prototype: ylimall(y0, y1, fign)
% Inputs: y0, y1 - ylim value.
%         fign - figure NO.
%
% See also  xlimall, gridall, scft0, xygo, mylegend.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/06/2023
    if nargin<3, h=gcf; else, h=figure(fign); end
    if nargin<2, 
        if length(y0)<2, y1=abs(y0); y0=-y1;
        else             y1=y0(2);   y0=y0(1);   end
    end
    ax = findall(h, 'type', 'axes');
    for k=1:length(ax)
        if isempty(get(ax(k),'Tag')), ylim(ax(k), [y0, y1]); end  % not for legend
    end
