function td = xdelay(x0, x1, t0, t1, ts)
% Odometer analysis.
%
% Prototype: td = xdelay(x0, x1, t0, t1, ts)
% Inputs: x0,x1 - data set with form [data, t].
%         t0,t1 - ananylsis within time interval [t0,t1]
% Output: td - time-delay x1 w.r.t. v0
%
% Example:
%   t=(0:0.01:1)'; x0=[sin(2*pi*10*t),t]; x1=[sin(2*pi*10*t),t-0.02];
%   xdelay(x0,x1);
%
% See also  odanalysis.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 24/11/2020
    if nargin<5, ts = 0.001; end
    if nargin<4, t1 = min(x0(end,2),x1(end,2)); end
    if nargin<3, t0 = max(x0(1,2),x1(1,2)); end
    t = (t0:ts:t1)';
    x00 = interp1(x0(:,2), x0(:,1), t, 'linear'); 
    x11 = interp1(x1(:,2), x1(:,1), t, 'linear'); 
    x0 = datacut(x0, t0,t1);
    x1 = datacut(x1, t0,t1);
    n = fix(min(t(end)-t(1),1)/ts);
    [r, lag]  = xcorr(x00, x11, fix(n/2), 'unbiased');   % r = r-r(1);
    [~, m] = max(r);
    td = lag(m)*ts;
    myfigure,
    subplot(211), plot(t, x00, t, x11); xygo('x'); legend('x0', 'x1');
    hold on, plot(x0(:,2), x0(:,1), '*', x1(:,2), x1(:,1), '*');
    subplot(212), plot(lag*ts, r/r(m), '-*', td, 1, 'or'); xygo('\tau / s', '\rho')
    title(sprintf('\\tau = %.2f (ms)', td*1000));
    
