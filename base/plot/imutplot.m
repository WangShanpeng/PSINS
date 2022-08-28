function imut = imutplot(imut, n)
% IMU & Temperature plot.
%
% Prototype: imutplot(temp, n)
% Inputs: imut - [imu, temp, t] array
%         n - mean count
% Output: imut - cumsum IMU & mean Temp oputput
%
% See also  imuplot, imumeanplot, imutemplot, templot.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/07/2021
global glv
    if nargin<2, n=1; end
    nts = n*diff(imut(1:2,end));
    imut = [meann(imut(:,1:end-1),n),imut(n:n:end,end)];
    imut(:,1:6) = imut(:,1:6)*n;
    [imu, b] = delbias(imut(:,1:6));
    b = [b(1:3)/nts/glv.dph; b(4:6)/nts/glv.g0];
    myfig;
    tscalepush('t/m');
    t = imut(:,end)/tscaleget();
	subplot(3,1,[1,2]), ax=plotyy(t, imu(:,1:3)/nts/glv.dph, t, imu(:,4:6)/nts/glv.ug);
    xyygo(ax, 'wdph', 'fug'); legend('Gx', 'Gy', 'Gz', 'Ax', 'Ay', 'Az');
    title(['mean=', sprintf('%f; ', b), '(\circ/h,g)']);
	subplot(313), plot(t, imut(:,7:end-1));  xygo('Temp');
    tscalepop();
