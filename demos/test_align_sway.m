% Ship-swaying-base alignment simulation.
% See also test_align_rotation, test_align_transfer. 
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/02/2022
glvs
ts = 1/100;
%% imu simu
att0 = [0;0;0]*glv.deg;
pos0 = posset(34, 116, 480);
imuerr = imuerrset(0.01, 100, 0.0001, .0);
[imu, avp0, avp] = imusway([att0;pos0], [3;5;2]*glv.deg, [12;15;30], ts, 1800, imuerr); 
% imuplot(imu);  insplot(avp);
%% align
phi = [.02;.02;.1]*glv.deg;  att0 = q2att(qaddphi(a2qua(att0),phi));
wvn = [0.01;0.01;0.01];
[att00, attk] = alignvn(imu, att0, pos0, phi*10, imuerr, wvn);
%% error comparison
avpcmpplot(attk, avp(:,[1:3,end]), 'phi');
