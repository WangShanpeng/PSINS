function xy = rotline(ang, n, s)
% Rotate line:
%
% Prototype: xy = rotline(ang, n)
% Inputs: ang - rotation angle
%         n - n^th line in the current axis
%         s - enlarge scale
% Output: xy - x,y data after rotation
%
% See also lmrs, lmc.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 24/09/2024
    if nargin<3, s=1; end
    if nargin<2, n=1; end
    lines = findall(gca, 'type', 'line');
    hl = lines(length(lines)-n+1);
    x = get(hl, 'xdata')*s;
    y = get(hl, 'ydata')*s;
    xx = x*cos(ang) -  y*sin(ang);
	yy = x*sin(ang) +  y*cos(ang);
    set(hl, 'xdata', xx);
    set(hl, 'ydata', yy);
    xy = [xx', yy'];

