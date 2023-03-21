function ttest(t, xist)
% Continuity test for time flag.
%
% Prototype: ttest(t)
% Inputs: t - time stamp
%         xist - x-axis is t
%
% See also  sortt, tsetflag, tshift, tsyn, igsplot.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/01/2021
    if nargin<2, xist=0; end
    t = t(:,end);  dt = diff(t);
    myfigure,
    subplot(211), plot(t-t(1));  xygo('k / dt', 't / s');
    if xist,
        subplot(212), plot(t(2:end), dt); xygo('dt / s');
    else
        subplot(212), plot(dt); xygo('k / dt', 'dt / s');
    end
    title(sprintf('mean(dt) = %f (s)', mean(dt)));

