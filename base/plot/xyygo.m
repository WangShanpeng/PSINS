function xyygo(ax, xtext, ytext1, ytext2)
% for plotyy, Xlable 'xtext', Ylabel 'ytext1','ytext2' & Grid On
%
% Prototype: xyygo(ax, xtext, ytext1, ytext2)
% Inputs: ax - axis handle return from plotyy
%         xtext, ytext1, ytext2 - text labels to show in figure x-axis & y-axis, 
%
% See also  xygo, labeldef.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/02/2014
    if nargin==3 % xyygo(ax, ytext1, ytext2)
        ytext2 = ytext1;
        ytext1 = xtext;
        xtext = labeldef('t');
    end
	xlabel(labeldef(xtext));
    set(get(ax(1),'Ylabel'), 'String', labeldef(ytext1));
    set(get(ax(2),'Ylabel'), 'String', labeldef(ytext2));
    grid on;  hold on;
