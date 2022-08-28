% Large yaw misalignment angle error propagation test.
% See also  test_align_ekf, Jacob5.
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/07/2022
glvs
[nn, ts, nts] = nnts(2, 0.1);
T = 600;
avp0 = avpset([0;0;-30], [0;0;0], 34);  qnb0 = a2qua(avp0(1:3)');
imuerr = imuerrset(0.01, 50, 0.001, 5);
[imu, eth] = imustatic(avp0, ts, T, imuerr);  Cnn=rv2m(-eth.wnie*nts/2); % imu simulation
phi0 = [1.0; 1.0; 30*60]*glv.min; % large yaw misalignment error
qnb = qaddphi(qnb0, phi0);  vn = zeros(3,1); % nav parameters init
tpara.wnie = eth.wnie; tpara.fn = -eth.gn; tpara.ts = nts;
len = length(imu); res = prealloc(fix(len/nn),11);
Xk = [phi0; vn(1:2)];
ki = timebar(nn, len, 'Jacob5 test.' );
for k=1:nn:len-nn+1
    k1 = k+nn-1;
    wvm = imu(k:k1,1:6);  t = imu(k1,7);
    [phim, dvbm] = cnscl(wvm);
    Cnb = q2mat(qnb);
    dvn = Cnn*Cnb*dvbm;  vn = vn + dvn + eth.gn*nts;   % velocity updating
    qnb = qupdt(qnb, phim-Cnb'*eth.wnie*nts);  % attitude updating
    tpara.fn = dvn/nts;
    [Phikk_1, Xk] = Jacob5(Xk, tpara);
    res(ki,:) = [qq2afa(qnb,qnb0); vn(1:2); Xk; t]';
    ki = timebar;
end
myfig;
subplot(311), plot(res(:,end), res(:,[1:2,6:7])/glv.sec); xygo('phiEN');
subplot(312), plot(res(:,end), res(:,[3,8])/glv.min); xygo('phiU');
subplot(313), plot(res(:,end), res(:,[4:5,9:10])); xygo('dV');
