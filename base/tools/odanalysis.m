function kod = odanalysis(od, avp)
% Odometer analysis.
%
% Prototype: kod = odanalysis(od, avp)
% Inputs: od - [od_increment, t].
%         avp - SINS avp output
% Output: kod - od scale factor
%
% See also  odplot, drupdate.

% Copyright(c) 2009-2019, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/12/2019
global glv
    vod = odplot(od, 100, 0);
    ts = mean(diff(od(:,end)));
    tins = interp1(od(:,end), od(:,end), avp(1,end):ts:avp(end,end));
    tins(isnan(tins)) = [];
    vins = interp1(avp(:,end), normv(avp(:,4:6)), tins);
    tod = interp1(tins, tins, od(:,end));
    tod(isnan(tod)) = [];
    vod = interp1(od(:,end), vod, tod);  % figure, plot(tins, vins, tod, vod*sum(vins)/sum(vod));
    n = 100;
    [r, lag]  = xcorr(vins, vod, n);   % r = r-r(1);
    ts = mean(diff(tod));
    [~, m] = max(r); mn = m-n-1;
    myfigure,
    subplot(221), plot(lag*ts, r/r(m), '-*', lag(m)*ts, 1, 'or'); xygo('\tau / s', '\rho')
    idx = vins>5;
    kod = sum(vins(idx))/sum(vod(idx(1:length(vod))));
    vod = kod*vod;
    dyaw = diff(avp(:,3))/0.01/glv.deg; h=5; idx = dyaw>h; dyaw(idx) = 200;  idx = dyaw<-h; dyaw(idx) = -200;
    subplot(222), plot(tins(1+100:end-mn-100), vins(mn+1+100:end-100), tod, vod); xygo('vel / m/s');
    legend('Vins', 'Vod');
    vod1 = interp1(tins(1+100:end-mn-100), vins(mn+1+100:end-100), tod);
    idx = ~isnan(vod1);
    subplot(223), plot(tod(idx), vod(idx)-vod1(idx), avp(1:end-1,end), dyaw*0.001-h*0.1); xygo('vel / m/s');
    legend('Vod-Vins', 'dyaw');
    subplot(224), plot(avp(1:end-1,end), avp(1:end-1,3)/glv.deg); xygo('y');
    
