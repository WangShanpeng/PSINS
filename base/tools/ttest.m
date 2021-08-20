function ttest(t)
% Continuity test for time flag.
%
% Prototype: ttest(t)
% Inputs: t - time stamp
%
% See also  sortt, tsetflag, tshift, tsyn, igsplot.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/01/2021
    t = t(:,end);  dt = diff(t);
    myfigure,
    subplot(211), plot(t-t(1));  xygo('k', 't / s');
    subplot(212), plot(dt); xygo('k', 'dt / s');
    title(sprintf('mean(dt) = %f (s)', mean(dt)));

