function xpplot(x, p, clm, unt, untstr, clm1, unt1, untstr1)
% Kalman filter xk/pk or zk/rk plot.
%
% Prototype: xpplot(x, p, clm, unt, untstr, ...)
% Inputs: x,p - Kalman xk/pk or zk/rk
%         clm - column index
%         unt - unit
%         untstr = unit string to show in ylabel
%
% See also  kfplot, xpclm, dataplot, inserrplot, insserrplot, kffile, rvpplot.

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
    if nargin<3, clm = 1:length(x)-1; end
    if ischar(clm), untstr=clm; clm=1; end   % xpplot(x, p, untstr)
    if ischar(unt), untstr=unt; unt=1; end   % xpplot(x, p, clm, untstr)
    if strcmp(untstr,'phi')
        if length(clm)<1, clm=1:3; end
        t = p(:,end); x = x(:,clm); p = p(:,clm);
        subplot(211), plot(t, x(:,1:3)/glv.min); xygo('phi');
        subplot(212), plot(t, sqrt(p(:,1:3))/glv.min); xygo('phi');
        return
    end
    if strcmp(untstr,'dvn')
        if length(clm)<1, clm=4:6; end
        t = p(:,end); x = x(:,clm); p = p(:,clm);
        subplot(211), plot(t, x(:,1:3)); xygo('V');
        subplot(212), plot(t, sqrt(p(:,1:3))); xygo('V');
        return
    end
    if strcmp(untstr,'dpos')
        if length(clm)<1, clm=4:6; end
        t = p(:,end); x = x(:,clm); p = p(:,clm);
        subplot(211), plot(t, [x(:,1:2)*glv.Re,x(:,3)]); xygo('dP');
        subplot(212), plot(t, [sqrt(p(:,1:2))*glv.Re,sqrt(p(:,3))]); xygo('dP');
        return
    end
    if strcmp(untstr,'ebdb')
        if length(clm)<7, clm=10:15; end
        t = p(:,end); x = x(:,clm); p = p(:,clm);
        subplot(221), plot(t, x(:,1:3)/glv.dph); xygo('eb');  title('Gyro drift')
        subplot(222), plot(t, x(:,4:6)/glv.ug); xygo('db'); title('Acc bias')
        subplot(223), plot(t, sqrt(p(:,1:3))/glv.dph); xygo('eb'); title('Std Gyro drift')
        subplot(224), plot(t, sqrt(p(:,4:6))/glv.ug); xygo('db'); title('Std Acc bias')
        return
    end
    if strcmp(untstr,'lever')
        if length(clm)<3, clm=16:18; end
        t = p(:,end); x = x(:,clm); p = p(:,clm);
        subplot(211), plot(t, x(:,1:3)); xygo('lever'); title('Lever arm')
        subplot(212), plot(t, sqrt(p(:,1:3))); xygo('lever'); title('std (Lever arm)')
        return
    end
    if strcmp(untstr,'dT')
        if length(clm)<1, clm=19; end
        t = p(:,end); x = x(:,clm); p = p(:,clm);
        subplot(211), plot(t, x(:,1)); xygo('dT');
        subplot(212), plot(t, sqrt(p(:,1))); xygo('dT');
        return
    end
    if strcmp(untstr,'dKG1')
        if length(clm)<1, clm=20; end
        t = p(:,end); x = x(:,clm); p = p(:,clm);
        subplot(211), plot(t, x(:,1)/glv.ppm); xygo('dKzz'); title('Scale Gyro')
        subplot(212), plot(t, sqrt(p(:,1))/glv.ppm); xygo('dKzz'); title('Std Scale Gyro')
        return
    end
    if strcmp(untstr,'dKG3')
        if length(clm)<3, clm=20:22; end
        t = p(:,end); x = x(:,clm); p = p(:,clm);
        subplot(221), plot(t, x(:,1:2)/glv.sec); xygo('dKx/yz'); title('Install angles')
        subplot(223), plot(t, sqrt(p(:,1:2))/glv.sec); xygo('dKx/yz'); title('Std Install angles')
        subplot(222), plot(t, x(:,3)/glv.ppm); xygo('dKzz'); title('Scale Gyro')
        subplot(224), plot(t, sqrt(p(:,3))/glv.ppm); xygo('dKzz'); title('Std Scale Gyro')
        return
    end
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
    if strcmp(untstr,'yaw')
        if length(clm)<1, clm=19; end
        t = p(:,end); x = x(:,clm); p = p(:,clm);
        subplot(211), plot(t, x(:,1)/glv.deg); xygo('y');
        subplot(212), plot(t, sqrt(p(:,1))/glv.deg); xygo('y');
        return
    end
    if strcmp(untstr,'mu')
        if length(clm)<1, clm=1:3; end
        t = p(:,end); x = x(:,clm); p = p(:,clm);
        subplot(211), plot(t, x(:,1:3)/glv.min); xygo('mu');
        subplot(212), plot(t, sqrt(p(:,1:3))/glv.min); xygo('mu');
        return
    end
    if strcmp(untstr,'kappa')
        if length(clm)<3, clm=16:18; end
        t = p(:,end); x = x(:,clm); p = p(:,clm);
        subplot(221), plot(t, x(:,[1,3])/glv.deg); xygo('dP,dY / \circ'); title('Pitch/Yaw Install Angle')
        subplot(222), plot(t, x(:,2)*1000); xygo('dKod / 0.1%'); title('OD Scale Error')
        subplot(223), plot(t, sqrt(p(:,[1,3]))/glv.deg); xygo('dP,dY / \circ'); title('Std P/Y')
        subplot(224), plot(t, sqrt(p(:,2))*1000); xygo('dKod / 0.1%'); title('Std OD Scale Error')
        return
    end
    subplot(211), plot(x(:,end), scaleclm(x(:,clm),1./unt)); xygo(sprintf('x / %s',untstr));
    subplot(212), plot(p(:,end), scaleclm(sqrt(p(:,clm)),1./unt)); xygo(sprintf('std(x) / %s',untstr));
