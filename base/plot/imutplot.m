function imut = imutplot(imut, n, typ)
% IMU & Temperature plot.
%
% Prototype: imutplot(temp, n)
% Inputs: imut - [imu, temp, t] array
%         n - mean count
%         typ - plot type
% Output: imut - cumsum IMU & mean Temp oputput
%
% See also  imuplot, imumeanplot, imutemplot, templot.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/07/2021
global glv
    if nargin<3, typ=1; end
    if nargin<2, n=1; end
    nts = n*diff(imut(1:2,end));
    imut = [meann(imut(:,1:end-1),n),imut(n:n:end,end)];
    imut(:,1:6) = imut(:,1:6)*n;
    [imu, b] = delbias(imut(:,1:6));
    b = [b(1:3)/nts/glv.dph, b(4:6)/nts/glv.g0];
    myfig;
    tscalepush('t/m');
    t = imut(:,end)/tscaleget();
    if typ==1
        subplot(3,1,[1,2]), ax=plotyy(t, imu(:,1:3)/nts/glv.dph, t, imu(:,4:6)/nts/glv.ug);
        xyygo(ax, 'wdph', 'fug'); legend('Gx', 'Gy', 'Gz', 'Ax', 'Ay', 'Az');
        title(['w0=', sprintf('%f; ', b(1:3)), '(\circ/h)', '     f0=', sprintf('%f; ', b(4:6)), '(g)']);
        subplot(313), plot(t, imut(:,7:end-1));  xygo('Temp');
    elseif typ==2
        subplot(321), plot(imut(:,7), imu(:,1)/nts/glv.dph);  xygo('Temp', 'wxdph');  title(['w0=', sprintf('%f; ', b(1:3)), '(\circ/h)']);
        subplot(323), plot(imut(:,7), imu(:,2)/nts/glv.dph);  xygo('Temp', 'wydph');
        subplot(325), plot(imut(:,7), imu(:,3)/nts/glv.dph);  xygo('Temp', 'wzdph');
        subplot(322), plot(imut(:,7), imu(:,4)/nts/glv.ug);  xygo('Temp', 'fxug');  title(['f0=', sprintf('%f; ', b(4:6)), '(g)']);
        subplot(324), plot(imut(:,7), imu(:,5)/nts/glv.ug);  xygo('Temp', 'fyug');
        subplot(326), plot(imut(:,7), imu(:,6)/nts/glv.ug);  xygo('Temp', 'fzug');
    end
    tscalepop();
