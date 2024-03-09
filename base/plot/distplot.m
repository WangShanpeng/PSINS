function distplot(dist)
% Distance plot.
%
% Prototype: distplot(dist)
% Inputs: dist - distance data
%
% See also  odplot, distance.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 14/01/2023
    myfig,
    if dist(end,1)>1e4
        plot(dist(:,2), dist(:,1)/1e4); xygo('dist / m');
    else
        plot(dist(:,2), dist(:,1)); xygo('dist / m');
    end
