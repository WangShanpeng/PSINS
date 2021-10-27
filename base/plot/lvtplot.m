function lvtplot(lvt)
% Lever & dT plot.
%
% Prototype: lvtplot(lvt)
% Input: lvt - [ lever dt t ] array
%
% See also  odpplot, xpplot.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/08/2021
    myfig;
    if size(lvt,2)==5
        subplot(211); plot(lvt(:,end), lvt(:,1:3)); xygo('L');
        subplot(212); plot(lvt(:,end), lvt(:,4)); xygo('dT');
    else
        plot(lvt(:,end), lvt(:,1:3)); xygo('L');
    end