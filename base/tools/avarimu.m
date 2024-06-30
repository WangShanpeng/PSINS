function [sigma, tau, m] = avarimu(imu)
% Calculate Allan variance for SIMU gyro & acc.
%
% Prototype: avarimu(imu)
% Input: imu - SIMU data
%
% Example
%     imu = imustatic(zeros(9,1), 0.01, 3600, imuerrset(0.01,100,0.01,1)); avarimu(imu);
%
% See also  avar, avars, avarfit.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 14/07/2023
global glv
    if size(imu,2)==4, imu=[imu(:,1:3),zeros(length(imu),3),imu(:,end)]; end;  % avarimu(gyro);
    sigma = []; tau = []; Err = []; m = [];
    ts = diff(imu(1:2,end));
    for k=1:3
        g = imu(:,k)/ts/glv.dph;
        m(k)=mean(g);  imu(:,k) = g-m(k);
        [sigma(:,k), tau(:,k), Err(:,k)] = avar(g, ts, 0, 0);
    end
    for k=4:6
        a = imu(:,k)/ts/glv.mg;
        m(k)=mean(a);  imu(:,k) = a-m(k);
        [sigma(:,k), tau(:,k), Err(:,k)] = avar(a, ts, 0, 0);
    end
    myfig;
   	subplot(221), plot(imu(:,end), imu(:,1:3));  xygo('\it\omega / \rm(\circ)/h');
    title(['Mean: ', sprintf('%.3f; ',[m(1:3),norm(m(1:3))]), ' \circ/h']);
   	subplot(223), loglog(tau(:,1), sigma(:,1:3));  xygo('\it\tau \rm/ s', '\it\sigma_A\rm( \tau ) /\rm (\circ)/h');
   	subplot(222), plot(imu(:,end), imu(:,4:6));  xygo('\itf ^b / \rmmg');
    title(['Mean: ', sprintf('%.3f; ',[m(4:6),norm(m(4:6))]),' mg']);
   	subplot(224), loglog(tau(:,1), sigma(:,4:6));  xygo('\it\tau \rm/ s', '\it\sigma_A\rm( \tau ) /\rm mg');
    m = m(:);

