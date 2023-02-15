function gridall(onoff, fign)
% In current figure, set all the axes grid on/off;
%
% Prototype: gridall(onoff)
% Inputs: onoff - 'on' or 'off'
%         fign - figure NO.
%
% See also  xlimall.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 18/12/2022
    if nargin<2, h=gcf; else, h=figure(fign); end
    if nargin<1, onoff='on'; end
    if isnumeric(onoff)
        if onoff==1, onoff='on'; elseif onoff==0, onoff='off'; end
    end
    ax = findall(h, 'type', 'axes');
    for k=1:length(ax)
        if isempty(get(ax(k),'Tag')), grid(ax(k), onoff); end  % not for legend
    end
