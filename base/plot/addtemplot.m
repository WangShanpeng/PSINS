function addtemplot(temp)
% Add temperature plot to imumeanplot.
%
% Prototype: addyawplot(yaw)
% Input: temp - [temp,t] array. 
%
% See also  imumeanplot, addyawplot.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 31/07/2021
    subplot(326);
    h = findobj(gca, 'type', 'line');
    x=get(h(1),'xdata'); y=get(h(1),'ydata');
    cla;
    ax = plotyy(x,y, temp(:,end), temp(:,1:end-1)); 
    xyygo(ax, 'fz', 'Temp');