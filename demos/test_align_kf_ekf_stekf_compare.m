% KF,EKF & STEKF align methods are compared.
% See also  alignvn_ekf, alignvn_stekf, test_align_methods_compare.
% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/10/2021
glvs;
ts = 0.02; 
attr = [0;0;20]*glv.deg;  pos0 = glv.pos0;
imuerr = imuerrset(0.03, 100, 0.001, 1);
imu = imustatic([attr;pos0], ts, 600, imuerr);  % imuplot(imu);
%%
phi = 5*[2; 3; 18]*glv.deg;      % Please modify here, '0.1*, 1.0*, 5.0* or 10.0*'
att = qaddafa(a2qua(attr),phi);
phi0 = [5; 5; 30]*glv.deg; wvm = 0.01; isfig = 0;
[att0, attk, xkpk] = alignvn(imu, att, pos0, phi0, imuerr, wvm, isfig);
[att1, attk1, xkpk1] = alignvn_ekf(imu, att, pos0, phi0, imuerr, wvm, isfig);
% [att1, attk1, xkpk1] = alignvn_stekf0(imu, att, pos0, phi0, imuerr, wvm, isfig); xkpk1=xkpk1(:,[1:5,13:17,end]);
[att2, attk2, xkpk2] = alignvn_stekf(imu, att, pos0, phi0, imuerr, wvm, isfig);
%%
myfig,
attr0 = repmat(attr', length(attk2), 1);
subplot(321), plot(attk(:,end), attk(:,1)/glv.deg, attk1(:,end), attk1(:,1)/glv.deg, attk2(:,end), [attk2(:,1),attr0(:,1)]/glv.deg);  xygo('p');  title('Attitude');
legend('KF', 'EKF', 'STEKF', 'True Att');
subplot(323), plot(attk(:,end), attk(:,2)/glv.deg, attk1(:,end), attk1(:,2)/glv.deg, attk2(:,end), [attk2(:,2),attr0(:,2)]/glv.deg);  xygo('r');
subplot(325), plot(attk(:,end), attk(:,3)/glv.deg, attk1(:,end), attk1(:,3)/glv.deg, attk2(:,end), [attk2(:,3),attr0(:,3)]/glv.deg);  xygo('y');
subplot(322), plot(xkpk(:,end), sqrt(xkpk(:,13))/glv.deg, xkpk1(:,end), sqrt(xkpk1(:,6))/glv.deg, xkpk2(:,end), sqrt(xkpk2(:,13))/glv.deg);  xygo('phiE');  title('STD-\it\phi')
subplot(324), plot(xkpk(:,end), sqrt(xkpk(:,14))/glv.deg, xkpk1(:,end), sqrt(xkpk1(:,7))/glv.deg, xkpk2(:,end), sqrt(xkpk2(:,14))/glv.deg);  xygo('phiN');
subplot(326), plot(xkpk(:,end), sqrt(xkpk(:,15))/glv.deg, xkpk1(:,end), sqrt(xkpk1(:,8))/glv.deg, xkpk2(:,end), sqrt(xkpk2(:,15))/glv.deg);  xygo('phiU');

