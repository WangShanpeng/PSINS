% SINS error simulation rotated by single axis.
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/011/2022
glvs
ts = 0.01;  T = 30;
att0 = [0; 0; 0]*glv.deg;  pos0 = posset(34,0,0);
A = 14*glv.deg; f = 0.5;
ang = [zeros(1000,1); sin(2*pi*(0:ts:fix(T/f))'); zeros(1000,1)];
[b, a] = ar1coefs(ts, 0.1); ang = filtfilt(b, a, ang);  ang=A/max(ang)*ang;
rot = [1; 0; 0.9];  rot = rot/norm(rot);
Cnb = a2mat(att0);  att = zeros(length(ang),4);
for k=1:length(ang);
    att(k,:) = [m2att(Cnb*rv2m(rot*ang(k))); k*ts]';
end
% insplot(att,'a');
[imu, avp0] = avp2imu(att,pos0);
% imuplot(imu,1);
Kg = [1.03, 1100*glv.sec, -2000*glv.sec;
      0, 1.0004, 3000*glv.sec;
      0, 0, 1.01];
imu(:,1:3) = imu(:,1:3)*Kg';
avp = inspure(imu, avp0, 'f');
avpcmpplot(att, avp(:,[1:3,end]),'a');


