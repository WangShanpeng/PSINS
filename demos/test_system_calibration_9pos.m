% 9-rotation-postion systematic calibration simulation.
% Ref: “惯性仪器测试与数据分析”10.4节.
% See also test_system_calibration_3_axis_turntable, test_system_calibration_19pos
% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/08/2024
glvs
ts = 0.01;
att0 = [1; 2; 3]*glv.deg/3;  pos0 = llh(34,0,400);
paras = [
    1    0,0,1 180, 9, 40,20  % ZU
    2    1,0,0 180, 9, 20,20  % ZD
    3    0,1,0  90, 9, 10,10
    4    1,0,0 180, 9, 20,20  % XU
    5    0,1,0 180, 9, 20,20  % XD
    6    0,0,1  90, 9, 10,10
    7    0,1,0 180, 9, 20,20  % YU
    8    0,0,1 180, 9, 20,40  % YD
%     9    1,0,0  90, 9, 20,40  % ZU
];  paras(:,5) = paras(:,5)*glv.deg;
att = attrottt(att0, paras, ts);
imu = avp2imu(att,pos0);
imuplot(imu,1);
imu1 = imuclbt(imu);
[clbt, av] = sysclbt(imu1, pos0);
