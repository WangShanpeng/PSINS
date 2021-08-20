function clbtplot(xk, xorp)
% Calibration KF result plot function
%
% Prototype: clbtplot(xk, xorp)
% Inputs: xk - Xk or Pk to plot, 
%         xorp - 'x' or 'p'
%
% See also  labeldef, kfplot.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/03/2020
global glv
    if nargin<2,
        if min(min(xk(:,1:end-1)))<0, xorp = 'x';
        else, xorp = 'p';
        end
    end
    if xorp=='p', xk(:,1:end-1)=sqrt(xk(:,1:end-1)); end
    myfigure
    t = xk(:,end);
    subplot(331), plot(t, xk(:,1:3)/glv.min); xygo('phi');
    subplot(332), plot(t, xk(:,4:6)); xygo('dV')
    subplot(333), plot(t, xk(:,7:9)/glv.dph); xygo('eb');
    subplot(334), plot(t, xk(:,10:12)/glv.ug); xygo('db');
    subplot(335), plot(t, xk(:,13:4:21)/glv.ppm); xygo('dKii / ppm');
        hold on,  plot(t, xk(:,22:4:30)/glv.ppm, '--');
    subplot(336), plot(t, xk(:,[14:16,18:20])/glv.sec); xygo('dKij / (\prime\prime)');
    	hold on,  plot(t, xk(:,[23:25,27:29])/glv.sec, '--');
    subplot(337), plot(t, xk(:,31:33)/glv.ugpg2); xygo('Ka2 / ug/g^2');
    subplot(338), plot(t, xk(:,34:36)); xygo('lever arm / m');
    	hold on,  plot(t, xk(:,37:39),'--'); plot(t, xk(:,40:42), '-.');
    subplot(339), plot(t, xk(:,43)); xygo('\tau_{GA} / s');
