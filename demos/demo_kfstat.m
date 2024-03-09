% Demo for Kalman filter error distribution analysis and statistic.
%   Model:  dx/dt = | 0 -1 | * x + | 0 | * w
%                   | 0  0 |       | 1 |
%           zk = [ 1  0 ] * xk + vk
% See also  test_SINS_GPS_153_kfstat
% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/11/2023
    glvs;
    Ft = [0 -1; 0 0];    Gt = [0; 1];    qt = 0.1;
    Hk = [1 0];   rk = 10;
    %% trajectory simu
    kf = [];   Ts = 0.1;
    kf.Ft = Ft;   kf.Phikk_1 = eye(2)+Ft*Ts;  kf.Gammak = Gt;  kf.Qk = qt*Ts;
    kf.Hk = Hk;   kf.Rk = rk^2;
    x0 = [1000; 20];
    len = fix(x0(1)/x0(2)/Ts);
    xk = zeros(len,2);  zk = zeros(len,1);
    xk(1,:) = x0;
    zk(1) = kf.Hk*x0 + sqrt(kf.Rk)*randn(1);
    for k=2:len
        xk(k,:) = (kf.Phikk_1*xk(k-1,:)' + kf.Gammak*sqrt(kf.Qk)*randn(1))';
        zk(k) = kf.Hk*xk(k,:)' + sqrt(kf.Rk)*randn(1);
    end
    %% kf
    kf.xk = x0+[100;10];
    P0 = diag([100, 10])^2;   kf.Pxk = P0;
    kf = kfinit0(kf, Ts);
    xkest = xk;  obs=zeros(len,2);  pqr=zeros(2,4,len);
    kfs = kfstat([], kf);
    for k=1:len
        kf = kfupdate(kf, zk(k));
        xkest(k,:) = kf.xk';
        obs(k,:) = sqrt([P0(1,1)/kf.Pxk(1,1), P0(2,2)/kf.Pxk(2,2)]);
        kfs = kfstat(kfs, kf, 'B');
        kfs = kfstat(kfs);
        pqr(:,:,k) = [kfs.p, kfs.q, kfs.r];
    end
    %% plot
	myfig;  t = (1:len)'*Ts;  pqr=permute(pqr, [3,2,1]);
    subplot(221),  plot(t, [xk(:,1), zk, xkest(:,1)]);  xygo('distance / m');
    subplot(222),  plot(t, [xk(:,2), xkest(:,2)]);  xygo('vel / m/s');
    subplot(234),  semilogy(t, obs);  xygo('Observibility');  legend('x1', 'x2');
    subplot(235),  plot(t, pqr(:,:,1));  xygo('distance err scr');  legend('p11', 'p22', 'q11', 'r11');
    subplot(236),  plot(t, pqr(:,:,2));  xygo('vel err scr');  legend('p11', 'p22', 'q11', 'r11');
    