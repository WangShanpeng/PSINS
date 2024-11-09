% 2-axis Rotational SINS(RSINS) simulation.
% See also test_RSINS_1_axis.m
% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/09/2024
glvs
ts = 0.1; s=6*1.315*2; RT = 20*s; ST = 10*s;  % sampling, rotational & static interval
att0 = pry(0.1, 0.1, -0.1);  pos0 = posset(34,0,0);
paras = [
    1    0,0,1,  180, RT, ST, ST
    2    1,0,0, -180, RT, ST, ST
    3    0,0,1,  180, RT, ST, ST
    4    1,0,0, -180, RT, ST, ST
    5    1,0,0, -180, RT, ST, ST
    6    0,0,1,  180, RT, ST, ST
    7    1,0,0, -180, RT, ST, ST
    8    0,0,1,  180, RT, ST, ST  % 8
    9    0,0,1, -180, RT, ST, ST
   10    1,0,0,  180, RT, ST, ST
   11    0,0,1, -180, RT, ST, ST
   12    1,0,0,  180, RT, ST, ST
   13    1,0,0,  180, RT, ST, ST
   14    0,0,1, -180, RT, ST, ST
   15    1,0,0,  180, RT, ST, ST
   16    0,0,1, -180, RT, ST, ST  % 16
];  paras(:,5) = paras(:,5)*glv.deg;
paras(9:16,:)=paras([10,9,12,11,14,13,16,15],:); % improved-16 scheme
paras(:,2:3) = paras(:,[3,2]); % change to Y-Z
paras = repmat(paras, 1, 1);
% paras = [[0  0,0,1, 0, 1, ST-1, ST]; paras; [0  0,0,1, 0, 1, ST-1, ST]];
att = attrottt(att0, paras, ts);
% IMU simulation
len = length(att);
[imu, avp0] = avp2imu(att, pos0);
imuplot(imu,1);
% INS simulation
imuerr = imuerrset([1;1;1]*0.01, 100);
% avp = inspure(imuadderr(imu,imuerr),avp0,'f');
% avpcmpplot(att, avp(:,[1:3,end]), 'a');
avp = rsinsst(imuadderr(imu,imuerr), avp0, 3600*24);
