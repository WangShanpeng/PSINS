% 1-axis Rotational SINS(RSINS) simulation.
% See also test_RSINS_2_axis.m
% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/09/2024
glvs
ts = 0.1; RT = 30; ST = 30;  % sampling, rotational & static interval
att0 = pry(0, 0, -0.1);  pos0 = posset(34,0,0);
paras = [
    1    0,0,1,  180, RT, ST, ST
    2    0,0,1, -180, RT, ST, ST
    3    0,0,1, -180, RT, ST, ST
    4    0,0,1,  180, RT, ST, ST
];  paras(:,5) = paras(:,5)*glv.deg;
paras = repmat(paras, 1, 1);
% paras = [[0  0,0,1, 0, 1, ST-1, ST]; paras; [0  0,0,1, 0, 1, ST-1, ST]];
att = attrottt(att0, paras, ts);
% IMU simulation
len = length(att);
[imu, avp0] = avp2imu(att, pos0);
imuplot(imu,1);
% INS simulation
imuerr = imuerrset(0.01,100,0.000,0,  0,0,0,0,  0,0, 0,0);
% avp = inspure(imuadderr(imu,imuerr),avp0,'V');
% avpcmpplot(att, avp(:,[1:3,end]), 'a');
avp = rsinsst(imuadderr(imu,imuerr), avp0, 3600*24);
