function [yerr, y0, p] = deltrend(y, order, isfig)
% Delete trend.
%
% Prototype:  [yerr, y0] = deltrend(y, order)
% Inputs: y - input data
%         order - polyfit order, or polynomial coefficients array
%         isfig - figure flag
% Outputs: yerr,y0 - y=y0+yerr, y0 to be the trend
%          p - polynomial coefficients
%
% See also  delbias.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 10/05/2022 
    if nargin<2, order=1; end
    if nargin<3, isfig=0; end
    if size(y,2)>1,   % if the last clumn is x
        x = y(:,end); m = size(y,2)-1;
    else
        x = (1:size(y,1))'; m = 1;
    end
    y0 = y; yerr = y;
    for k=1:m
        if length(order)>1, p = order;
        else, p = polyfit(x,y(:,k),order); end
        y0(:,k) = polyval(p,x);
        yerr(:,k) = y(:,k) - y0(:,k);
    end
    if isfig==1
        myfig
        subplot(211), plot([y0,y]); xygo('k', 'val'); legend('trend', 'data');
        subplot(212), plot(yerr); xygo('k', 'val')
    end