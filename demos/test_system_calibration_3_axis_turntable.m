% 3-axis-turntable systematic calibration simulation.
% Ref: 惯性仪器测试与数据分析，表10.4-1.
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/06/2022
glvs
ts = 0.01;
att0 = [0; 0; 0]*glv.deg;  pos0 = posset(34,0,0);
n = 1;
n21pi = (2*n+1)*180;
paras = [
%     1    0,1,0, n21pi, 36*n+9, 60, 10  % method-1
%     2    0,1,0, 180, 19, 10, 10
%     3    0,0,1, 180, 19, 10, 10
%     4    1,0,0, 90, 9, 1, 1
%     5    0,1,0, -90, 9, 1, 1
%     6    1,0,0, n21pi, 36*n+9, 30, 10
%     7    1,0,0, 180, 19, 10, 10
%     8    0,1,0, 180, 19, 10, 10
%     9    0,0,1, 90, 9, 1, 1
%    10    1,0,0, -90, 9, 1, 1
%    11    0,0,1, n21pi, 36*n+9, 30, 10
%    12    0,0,1, 180, 19, 10, 10
%    13    1,0,0, 180, 19, 10, 10
%    14    0,1,0, 90, 9, 1, 1
%    15    0,0,1, 90, 9, 1, 60
    1    0,1,0, n21pi, 36*n+9, 60, 15  % method-2
    2    0,0,1, 180, 19, 15, 30
    3    0,1,0, 90, 9, 1, 1
    4    0,0,1, n21pi, 36*n+9, 30, 15
    5    1,0,0, 180, 19, 15, 30
    6    0,0,1, 90, 9, 1, 1
    7    1,0,0, n21pi, 36*n+9, 30, 15
    8    0,1,0, 180, 19, 15, 30
    9    0,0,1, 90, 9, 1, 1
   10    0,1,0, -90, 9, 1, 60
];  paras(:,5) = paras(:,5)*glv.deg;
att = attrottt(att0, paras, ts);
% att1=att; att1(:,end)=att1(:,end)/2; att3ddemo(att1(1:10:end,:),1);
% IMU simulation
imu = avp2imu(att,pos0);
imuplot(imu,1);
% systematic calibration
imu1 = imuclbt(imu);
[clbt, av] = sysclbt(imu1, pos0);
clbtfile(clbt);
% clbtfile(clbt, 'clbt19.bin');
% binfile('imu19.bin', [imu1,imu1(:,end)]);

