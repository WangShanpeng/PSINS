function lost = imuplot(imu, type, t0)
% SIMU data plot.
%
% Prototype: lost = imuplot(imu, type, t0)
% Inputs: imu - SIMU data, the last column is time tag
%         type - figure type
%         t0 - plot time t0 as 0
% Output: lost - lost index      
%          
% See also  imumeanplot, imutplot, cumangplot, insplot, inserrplot, kfplot, gpsplot, odplot, imuodplot, magplot, igsplot, ttest.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/08/2013, 09/02/2014, 03/10/2014
global glv
    dps = glv.dps; g0 = glv.g0;
    if ischar(imu)  % imu = imuplot('imu.bin', binfileClm);
        if nargin<2, type=7; end
        lost = binfile(imu, type); lost = lost(:,[1:6,end]);
        imuplot(lost);
        return;
    end
    if iscell(imu)
        for k=1:length(imu), imuplot(imu{k}); end
        return;
    end
    if size(imu,2)>10  % 5-axis redundancy IMU plot
        myfig;
        ts = diff(imu(1:2,end));  t = imu(:,end);
        subplot(221), plot(t, imu(:,1:3)/ts/dps); xygo('w');  title('X_R Y_F Z_U');
        subplot(223), plot(t, imu(:,4:5)/ts/dps); xygo('wS/T');
        subplot(222), plot(t, imu(:,6:8)/ts/g0);  xygo('f');
        subplot(224), plot(t, imu(:,9:10)/ts/g0);  xygo('fS/T');
        return;
    end
    if nargin<2, type=0; end
    if ischar(type)  % imuplot(imu, 't/h')
        tscalepush(type);
        imuplot(imu/tscaleget());
        tscalepop();
        return;
    end
    if length(type)>=7  % err = imuplot(imu1, imu2);
        imuplot(imu);
        imu2 = type;   t = imu2(:,end);  ts = diff(imu2(1:2,end));
        subplot(321), plot(t, imu2(:,1)/ts/dps, 'r'); legend('IMU1','IMU2');  title('X_R Y_F Z_U');
        subplot(323), plot(t, imu2(:,2)/ts/dps, 'r');
        subplot(325), plot(t, imu2(:,3)/ts/dps, 'r');
        subplot(322), plot(t, imu2(:,4)/ts/g0, 'r');
        subplot(324), plot(t, imu2(:,5)/ts/g0, 'r');
        subplot(326), plot(t, imu2(:,6)/ts/g0, 'r');
        if imu(end,end)<imu2(1,end) && imu2(end,end)<imu(1,end)
            imu = interp1n(imu, imu2(:,end));
            [t,i1,i2] = intersect(imu(:,end), imu2(:,end));
            lost = [[imu(i1,1:6)-imu2(i2,1:6)], t];  % imuerr = imu1-imu2;
        end
        return;
    end
    if size(imu,2)==3, imu(:,4) = (1:length(imu))'; end
    if size(imu,2)==6, imu(:,7) = (1:length(imu))'; end
    t = imu(:,end);
    if nargin==3,  % t0
        if ischar(t0)
            if strcmp(t0,'t0'), t0=t(1);
            elseif strcmp(t0,'t1'), t0=t(end);  end
        end
        t = t-t0;
    end
% 	ts = mean(diff(t));
	ts = mean(diff(t(1:2)));
    t = t/tscaleget;
%     if norm(mean(imu(:,4:6)))<9.8/2   % if it's velocity increment
        imu(:,1:end-1)=imu(:,1:end-1)/ts;
%     end
    myfig;
    if type==1 || type==11
        subplot(121), plot(t, [imu(:,1:3)]/dps); xygo('w');  title('X_R Y_F Z_U');
        subplot(122), plot(t, [imu(:,4:6)]/g0);  xygo('f');
        if type==11
            subplot(121), plot(t, [imu(:,1:3),normv(imu(:,1:3))]/dps); xygo('w'); legend('Wx','Wy','Wz','|W|');  title('X_R Y_F Z_U');
            subplot(122), plot(t, [imu(:,4:6),normv(imu(:,4:6))]/g0);  xygo('f'); legend('Ax','Ay','Az','|A|');
        end
    elseif type==2
        ax = plotyy(t, imu(:,1:3)/dps, t, imu(:,4:6)/g0); xyygo(ax, 'w', 'f');
    elseif type==3
        subplot(311), ax = plotyy(t, imu(:,1)/dps, t, imu(:,4)/g0); xyygo(ax, 'wx', 'fx');  title('X_R Y_F Z_U');
        subplot(312), ax = plotyy(t, imu(:,2)/dps, t, imu(:,5)/g0); xyygo(ax, 'wy', 'fy');
        subplot(313), ax = plotyy(t, imu(:,3)/dps, t, imu(:,6)/g0); xyygo(ax, 'wz', 'fz');
%     elseif type==4
%         wfdot = diff([imu(1,:);imu])/ts;
%         subplot(321), plot(t, [imu(:,1),wfdot(:,1)]/dps); xygo('wx');
%         subplot(323), plot(t, [imu(:,2),wfdot(:,2)]/dps); xygo('wy');
%         subplot(325), plot(t, [imu(:,3),wfdot(:,3)]/dps); xygo('wz');
%         subplot(322), plot(t, [imu(:,4),wfdot(:,4)]/g0);  xygo('fx');
%         subplot(324), plot(t, [imu(:,5),wfdot(:,5)]/g0);  xygo('fy');
%         subplot(326), plot(t, [imu(:,6),wfdot(:,6)]/g0);  xygo('fz');
    elseif type==4
        wfdot = diff([imu(1,:);imu]);
        subplot(321), myplotyy([imu(:,1)/dps,wfdot(:,1)/dps/ts,t], 'wx', 'w_xdot/(dps/s)');  title('X_R Y_F Z_U');
        subplot(323), myplotyy([imu(:,2)/dps,wfdot(:,2)/dps/ts,t], 'wy', 'w_ydot/(dps/s)');
        subplot(325), myplotyy([imu(:,3)/dps,wfdot(:,3)/dps/ts,t], 'wy', 'w_zdot/(dps/s)');
        subplot(322), myplotyy([imu(:,4)/g0,wfdot(:,4)/g0/ts,t], 'fx', 'f_xdot/(g/s)');
        subplot(324), myplotyy([imu(:,5)/g0,wfdot(:,5)/g0/ts,t], 'fy', 'f_ydot/(g/s)');
        subplot(326), myplotyy([imu(:,6)/g0,wfdot(:,6)/g0/ts,t], 'fz', 'f_zdot/(g/s)');
    elseif type==5  % test if some data repeat
        wfdot = diff([imu(1,:)+111;imu]);
        subplot(321), plot(t, imu(:,1)/dps); xygo('wx');  title('X->R/Y->F/Z->U');
        repidx = find(wfdot(:,1)==0);  if ~isempty(repidx),  plot(t(repidx),imu(repidx,1)/dps,'ro');  end
        subplot(323), plot(t, imu(:,2)/dps); xygo('wy');
        repidx = find(wfdot(:,2)==0);  if ~isempty(repidx),  plot(t(repidx),imu(repidx,2)/dps,'ro');  end
        subplot(325), plot(t, imu(:,3)/dps); xygo('wz');
        repidx = find(wfdot(:,3)==0);  if ~isempty(repidx),  plot(t(repidx),imu(repidx,3)/dps,'ro');  end
        subplot(322), plot(t, imu(:,4)/g0);  xygo('fx');
        repidx = find(wfdot(:,4)==0);  if ~isempty(repidx),  plot(t(repidx),imu(repidx,4)/g0,'ro');  end
        subplot(324), plot(t, imu(:,5)/g0);  xygo('fy');
        repidx = find(wfdot(:,5)==0);  if ~isempty(repidx),  plot(t(repidx),imu(repidx,5)/g0,'ro');  end
        subplot(326), plot(t, imu(:,6)/g0);  xygo('fz');
        repidx = find(wfdot(:,6)==0);  if ~isempty(repidx),  plot(t(repidx),imu(repidx,6)/g0,'ro');  end
    elseif type==glv.dph
        dt = diff(t);
        lost = abs(dt)>mean(dt)*1.5; tlost = t(lost)+dt(1);
        subplot(321), plot(t, imu(:,1)/glv.dph, tlost,imu(lost,1)/glv.dph,'ro'); xygo('wxdph');  title('X_R Y_F Z_U');
        subplot(323), plot(t, imu(:,2)/glv.dph, tlost,imu(lost,2)/glv.dph,'ro'); xygo('wydph');
        subplot(325), plot(t, imu(:,3)/glv.dph, tlost,imu(lost,3)/glv.dph,'ro'); xygo('wzdph');
        subplot(322), plot(t, imu(:,4)/g0,  tlost,imu(lost,4)/g0,'ro');  xygo('fx');
        subplot(324), plot(t, imu(:,5)/g0,  tlost,imu(lost,5)/g0,'ro');  xygo('fy');
        subplot(326), plot(t, imu(:,6)/g0,  tlost,imu(lost,6)/g0,'ro');  xygo('fz');
        lost = find(lost)+1;
    elseif type==-1
        subplot(321), plot(t, imu(:,1)/dps); xygo('wx');  title('X_R Y_F Z_U');
        subplot(323), plot(t, imu(:,2)/dps); xygo('wy');
        subplot(325), plot(t, imu(:,3)/dps); xygo('wz');
        subplot(322), plot(t, imu(:,4)/g0);  xygo('fx');
        subplot(324), plot(t, imu(:,5)/g0);  xygo('fy');
        subplot(326), plot(t, imu(:,6)/g0);  xygo('fz');
        if size(imu,2)==8
            hold off;
            plotyy(t, imu(:,6)/g0, t, imu(:,7)*ts);  xygo('fz');
        end
    else % type==0
        dt = diff(t);
        lost = abs(dt)>mean(dt)*1.5; tlost = t(lost)+dt(1);
        if size(imu,2)==4  % plot acc or gyro only
            f = mean(normv(imu(:,1:3)));
            if f>9.0 && f<10.6  % if acc static
                subplot(311), plot(t, imu(:,1)/g0,  tlost,imu(lost,1)/g0,'ro');  xygo('fx');  title('X_R Y_F Z_U');
                subplot(312), plot(t, imu(:,2)/g0,  tlost,imu(lost,2)/g0,'ro');  xygo('fy');
                subplot(313), plot(t, imu(:,3)/g0,  tlost,imu(lost,3)/g0,'ro');  xygo('fz');
            else
                subplot(311), plot(t, imu(:,1)/dps, tlost,imu(lost,1)/dps,'ro'); xygo('wx');  title('X_R Y_F Z_U');
                subplot(312), plot(t, imu(:,2)/dps, tlost,imu(lost,2)/dps,'ro'); xygo('wy');
                subplot(313), plot(t, imu(:,3)/dps, tlost,imu(lost,3)/dps,'ro'); xygo('wz');
            end
        else
            subplot(321), plot(t, imu(:,1)/dps, tlost,imu(lost,1)/dps,'ro'); xygo('wx');  title('X_R Y_F Z_U');
            subplot(323), plot(t, imu(:,2)/dps, tlost,imu(lost,2)/dps,'ro'); xygo('wy');
            subplot(325), plot(t, imu(:,3)/dps, tlost,imu(lost,3)/dps,'ro'); xygo('wz');
            subplot(322), plot(t, imu(:,4)/g0,  tlost,imu(lost,4)/g0,'ro');  xygo('fx');
            subplot(324), plot(t, imu(:,5)/g0,  tlost,imu(lost,5)/g0,'ro');  xygo('fy');
            subplot(326), plot(t, imu(:,6)/g0,  tlost,imu(lost,6)/g0,'ro');  xygo('fz');
        end
        lost = find(lost)+1;
    end

