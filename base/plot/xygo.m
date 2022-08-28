function xygo(xtext, ytext)
% Xlable 'xtext', Ylabel 'ytext' & Grid On
%
% Prototype: xygo(xtext, ytext)
% Inputs: xtext, ytext - text labels to show in figure x-axis & y-axis, 
%             but if nargin==1, then the xtext will show in y-axis  
%             with time label shown defaultly in x-axis.
%
% See also  labeldef, xyygo, myfig, xlimall, nextlinestyle.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/02/2014
    if nargin==0 % xygo
        ytext = 'value';
        xtext = labeldef('t');
    end
    if nargin==1 % xygo(ytext)
        ytext = xtext;
        xtext = labeldef('t');
    end
	xlabel(labeldef(xtext));
    ylabel(labeldef(ytext));
    grid on;  hold on;
