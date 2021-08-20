function xpplot(x, p, clm, unt, untstr, clm1, unt1, untstr1)
% Kalman filter xk/pk or zk/rk plot.
%
% Prototype: xpplot(x, p, clm, unt, untstr, ...)
% Inputs: x,p - Kalman xk/pk or zk/rk
%         clm - column index
%         unt - unit
%         untstr = unit string to show in ylabel
%
% See also  kfplot, inserrplot, kffile.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 10/01/2021
global glv
    myfigure
    if nargin>5
        subplot(221), plot(x(:,end), x(:,clm)/unt); xygo(sprintf('x / %s',untstr));
        subplot(223), plot(p(:,end), sqrt(p(:,clm))/unt); xygo(sprintf('std(x) / %s',untstr));
        subplot(222), plot(x(:,end), x(:,clm1)/unt1); xygo(sprintf('x / %s',untstr1));
        subplot(224), plot(p(:,end), sqrt(p(:,clm1))/unt1); xygo(sprintf('std(x) / %s',untstr1));
        return;
    end
    if nargin<5, untstr = '*'; end
    if nargin<4, unt = 1; end
    if strcmp(untstr,'dKG')
        if length(clm)<9, clm=20:28; end
        t = p(:,end); x = x(:,clm); p = p(:,clm);
        subplot(221), plot(t, x(:,[1,5,9])/glv.ppm); xygo('dKii'); title('Scale Gyro')
        subplot(222), plot(t, x(:,[2:4,6:8])/glv.sec); xygo('dKij'); title('Non-orthodox Gyro')
        subplot(223), plot(t, sqrt(p(:,[1,5,9]))/glv.ppm); xygo('dKii'); title('Std Scale Gyro')
        subplot(224), plot(t, sqrt(p(:,[2:4,6:8]))/glv.sec); xygo('dKij'); title('Std Non-orthodox Gyro')
        return
    end
    if strcmp(untstr,'dKA')
        if length(clm)<6, clm=29:34; end
        t = p(:,end); x = x(:,clm); p = p(:,clm);
        subplot(221), plot(t, x(:,[1,4,6])/glv.ppm); xygo('dKii'); title('Scale Acc')
        subplot(222), plot(t, x(:,[2,3,5])/glv.sec); xygo('dKij'); title('Non-orthodox Acc')
        subplot(223), plot(t, sqrt(p(:,[1,4,6]))/glv.ppm); xygo('dKii'); title('Std Scale Acc')
        subplot(224), plot(t, sqrt(p(:,[2,3,5]))/glv.sec); xygo('dKij'); title('Std Non-orthodox Acc')
        return
    end
    if strcmp(untstr,'dKGA')
        if length(clm)<15, clm=20:34; end
        t = p(:,end); x = x(:,clm); p = p(:,clm);
        subplot(221), plot(t, x(:,[1,5,9, 10,13,15])/glv.ppm); xygo('dKii'); title('Scale Gyro/Acc')
        subplot(222), plot(t, x(:,[2:4,6:8, 11:12,14])/glv.sec); xygo('dKij'); title('Non-orthodox Gyro/Acc')
        subplot(223), plot(t, sqrt(p(:,[1,5,9, 10,13,15]))/glv.ppm); xygo('dKii'); title('Std Scale Gyro/Acc')
        subplot(224), plot(t, sqrt(p(:,[2:4,6:8, 11:12,14]))/glv.sec); xygo('dKij'); title('Std Non-orthodox Gyro/Acc')
        return
    end
    if strcmp(untstr,'kappa')
        if length(clm)<19, clm=16:18; end
        t = p(:,end); x = x(:,clm); p = p(:,clm);
        subplot(221), plot(t, x(:,[1,3])/glv.deg); xygo('dP,dY / \circ'); title('Pitch/Yaw Install Angle')
        subplot(222), plot(t, x(:,2)*1000); xygo('dKod / 0.1%'); title('OD Scale Error')
        subplot(223), plot(t, sqrt(p(:,[1,3]))/glv.deg); xygo('dP,dY / \circ'); title('Std P/Y')
        subplot(224), plot(t, sqrt(p(:,2))*1000); xygo('dKod / 0.1%'); title('Std OD Scale Error')
        return
    end
    subplot(211), plot(x(:,end), scaleclm(x(:,clm),1./unt)); xygo(sprintf('x / %s',untstr));
    subplot(212), plot(p(:,end), sqrt(scaleclm(p(:,clm),1./unt))); xygo(sprintf('std(x) / %s',untstr));
