% Freedom/Wander-azimuth SINS pure inertial navigation simulation. 
% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/10/2021
glvs
ts = 0.01;       % sampling interval
%% trajectory segment setting
avp0 = [[0;0;90]*glv.deg; [-100;0;0]; glv.pos0]; % init avp
xxx = [];
seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'uniform',      100);
trj = trjsimu(avp0, seg.wat, ts, 2);  % imuplot(trj.imu);  insplot(trj.avp);
%% error setting
imuerr = imuerrset(0.0, 00, 0.00, 0);
imu = imuadderr(trj.imu, imuerr);
davp0 = avperrset([0.5;0.5;5]*0, 0.1*0, [10;10;10]*0);
avp00 = avpadderr(trj.avp0, davp0);
%% north-slaved SINS
avp = inspure(imu, avp00, 'f');  % avpcmpplot(trj.avp, avp);
%% freedom/wander-azimuth SINS
len = length(trj.imu);
[nn, ts, nts] = nnts(2, diff(trj.imu(1:2,end)));
insf = insinit(avp00, ts);  insf.afa_f0 = 0;
avpf = zeros(fix(len/nn), 11);
insw = insf;  insw.afa_w0 = 0;  avpw = avpf;
ki = timebar(nn, len, 'Freedom/Wander-azimuth SINS processing.');
for k=1:nn:len
	k1 = k+nn-1;
	wvm = imu(k:k1, 1:6);  t = imu(k1,end);
	insf = insupdate_f(insf, wvm);   insf.pos(3) = avp0(9);
    avpf(ki,:) = [insf.avp; insf.afa_f; t]';
	insw = insupdate_w(insw, wvm);   insw.pos(3) = avp0(9);
    avpw(ki,:) = [insw.avp; insw.afa_w; t]';
    ki = timebar;
end
% avperrf = avpcmpplot(trj.avp, avpf(:,[1:9,11]));
% avperrw = avpcmpplot(trj.avp, avpw(:,[1:9,11]));
avperr = avpcmpplot(avpf(:,[1:9,11]), avpw(:,[1:9,11]));
myfig, plot(avpf(:,end), avpf(:,10)/glv.deg, avpw(:,end), avpw(:,10)/glv.deg); xygo('y'); legend('freedom azimuth','wander azimuth');
