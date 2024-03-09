function cumangplot(wm, unit)
% Plot the cumsum of gyro angular increment.
%
% Prototype: cumangplot(wm, unit)
% Inputs: wm - gyro angular increment without mean ( in rad )
%         unit - unit for plot
%
% See also  imuplot, cumint.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 18/7/2023
global glv
    if nargin<2, unit=glv.min; end
    for k=1:size(wm,2)-1
        wm(:,k) = cumsum(wm(:,k)-mean(wm(:,k)));
    end
    myfig;
    plot(wm(:,end), wm(:,1:end-1)/unit); xygo('\Delta\it\theta \rm/ (\prime)');
    