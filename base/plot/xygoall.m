function xygoall(xlab, varargin)
% In current figure (or fign), x/y-label & grid on for all axes.
%
% Prototype: xygoall(xlab, ylabs)
% Inputs: xlab, x-label.
%         ylabs - y-labels.
%
% See also  xlimall, gridall.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 11/12/2023
    ax = findall(gcf, 'type', 'axes');  len = 0;
    for k=1:length(ax)
        if isempty(get(ax(k),'Tag')), len=len+1; end  % not for legend
    end
    if nargin==1, 
        for k=1:len, ylab{k}='Val'; end
    elseif nargin==len
        ylab{1} = xlab;
        for k=2:len, ylab{k} = varargin{k-1};  end
        xlab = labeldef('t/s');
    else
        ylab = varargin;
    end
    ax = flipud(ax);
    for k=1:len
        if isempty(get(ax(k),'Tag')),
            xlabel(ax(k), xlab); ylabel(ax(k), ylab{k}); grid(ax(k), 'on');
        end  % not for legend
    end
