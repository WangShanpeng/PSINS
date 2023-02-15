function attplot(varargin)
% att plot.
%
% Prototype: attplot(varargin)
% Inputs: varargin - atts (more than one att), attplot(att1, att2, att3, ...)
%          
% See also  insplot.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 13/01/2023
global glv
	myfig; nextlinestyle(-1);
    for k=1:nargin
        att = varargin{k};
        subplot(311), plot(att(:,end), att(:,1)/glv.deg, nextlinestyle(1)), xygo('p');
        subplot(312), plot(att(:,end), att(:,2)/glv.deg, nextlinestyle(0)), xygo('r');
        subplot(313), plot(att(:,end), att(:,3)/glv.deg, nextlinestyle(0)), xygo('y');
    end
