% Extended Kalman filter(EKF) simulation with large yaw misalignment angle.
% See also  test_align_some_methods, test_align_ukf.
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/09/2013
glvs
psinstypedef('test_align_ekf_def');
[nn, ts, nts] = nnts(2, 0.1);
T = 600;
avp0 = avpset([0;0;-30], [0;0;0], 34);  qnb0 = a2qua(avp0(1:3)');
imuerr = imuerrset(0.01, 50, 0.001, 5);
[imu, eth] = imustatic(avp0, ts, T, imuerr);  Cnn=rv2m(-eth.wnie*nts/2); % imu simulation
phi0 = [10; 10; 30*60]*glv.min; % large yaw misalignment error
qnb = qaddphi(qnb0, phi0);  vn = zeros(3,1); % nav parameters init
kf = kfinit(nts, phi0, imuerr, eth);  % kf init
kf1 = kf; % kf1 init for EKF
kf1.fx = @Jacob5; kf1.px.wnie = eth.wnie; kf1.px.fn = -eth.gn; kf1.px.ts = nts;
len = length(imu); [reskf, resekf] = prealloc(fix(len/nn),5+1);
ki = timebar(nn, len, 'EKF initial align simulation.' );
for k=1:nn:len-nn+1
    k1 = k+nn-1;
    wvm = imu(k:k1,1:6);  t = imu(k1,7);
    [phim, dvbm] = cnscl(wvm);
    Cnb = q2mat(qnb);
    dvn = Cnn*Cnb*dvbm;  vn = vn + dvn + eth.gn*nts;   % velocity updating
    qnb = qupdt(qnb, phim-Cnb'*eth.wnie*nts);  % attitude updating
    phi = qq2afa(qnb, qnb0);
    % Kalman filter
    kf = kfupdate(kf, vn(1:2));
    reskf(ki,:) = [kf.xk(1:3)-phi; kf.xk(4:5); t]';
    % EKF filter
    kf1.px.fn = dvn/nts;
    kf1 = ekf(kf1, vn(1:2));
    resekf(ki,:) = [kf1.xk(1:3)-phi; kf1.xk(4:5); t]';
    ki = timebar;
end
kfplot(reskf, resekf);
