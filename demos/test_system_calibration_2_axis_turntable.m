% 2-axis-turntable(outer-pitch/inner-yaw) systematic calibration simulation.
% Ref: doc\系统级标定转台转动方案.docx
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/06/2022
glvs
ts = 0.01;
att0 = [0; 0; 0]*glv.deg;  pos0 = posset(34,0,0);
n21pi = (2*1+1)*180;
paras = [
    1    0,1,0, n21pi, 50, 60, 15
    2    0,0,1, 90, 9, 15, 15
    3    1,0,0, n21pi, 50, 15, 15
    4    0,0,1, 180, 19, 15, 15
    5    1,0,0, -90, 9, 15, 15
    6    0,0,1, n21pi, 50, 15, 15
    7    0,0,1, -90, 9, 15, 15
    8    0,0,1, 180, 19, 15, 15
    9    0,1,0, 90, 9, 15, 60
];  paras(:,5) = paras(:,5)*glv.deg;
att = attrottt(att0, paras, ts);
% att1=att; att1(:,end)=att1(:,end)/2; att3ddemo(att1(1:10:end,:),1);
% IMU simulation
len = length(att);
[imu, avp0] = avp2imu(att, pos0);
imuplot(imu,1);  % inspure(imu,avp0);
% systematic calibration
imu1 = imuclbt(imu);
[clbt, av] = sysclbt(imu1, pos0);
clbtfile(clbt);

