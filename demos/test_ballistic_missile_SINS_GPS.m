% Ballistic missile trajectory & SINS/GPS simulation
glvs
ts = 0.01;
%% trajectory simulation
% stage 1: 1/4 circle
T = 60;
t = (0:ts:T)';
afadot2 = 2*pi/2/T^2;
afa = 1/2*afadot2*t.^2;
r = 60000;
y = r-r*cos(afa);  z = r*sin(afa);
pitch = pi/2-afa;
tt = t;
% stage 2: leveling constant acceleration
v = (y(end)-y(end-1))/ts;
T = 120;
t = (ts:ts:T)';
a = -2;
y = [y; y(end)+v*t+1/2*a*t.^2];  z = [z; z(end)+t*0];
pitch = [pitch; t*0];
tt = [tt; tt(end)+t];
% stage 3: leveling constant velocity
v = (y(end)-y(end-1))/ts;
T = 120;
t = (ts:ts:T)';
a = -0;
y = [y; y(end)+v*t+1/2*a*t.^2];  z = [z; z(end)+t*0];
pitch = [pitch; t*0];
tt = [tt; tt(end)+t];
myfig, plot(y, z, '-'); xygo('front/m','up/m');
%% imu,avp,gps
dxyz = [y*0,y,z,tt];
dxyz(:,1:3) = ar1filt(dxyz(:,1:3), 100);
pos = dxyz2pos(dxyz);
ap = [pitch, zeros(length(pos),2), pos];
[imu, avp0, avp] = ap2imu(ap);
imuplot(imu);
gps = gpssimu(avp(1:100:end, :), 0.1, 10, 1,0,0, 0);
%% sins/gps
imuerr = imuerrset(0.15, 100, 0.001, 10);
davp = avperrset([1;1;3]*3, [1;1;1], [1;1;3]*100);
imu1 = imuadderr(imu,imuerr);
ins = insinit(avpadderr(avp0,davp), ts); ins.nts=ts;
avp1 = sinsgps(imu1, gps, ins, davp, imuerr);
avpcmpplot(avp, avp1(:,[1:9,end]), 'phi');


