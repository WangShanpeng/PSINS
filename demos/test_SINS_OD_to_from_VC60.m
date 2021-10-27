% SINS/OD integrated navigation simulation.
% See also  test_SINS_DR.
% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 14/09/2021
glvs
ts = 0.01;       % sampling interval
avp0 = [[0;0;0]; [0;0;0]; glv.pos0]; % init avp
% trajectory segment setting
xxx = [];
seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'uniform',      100);
seg = trjsegment(seg, 'accelerate',   10, xxx, 1);
seg = trjsegment(seg, 'uniform',      100);
seg = trjsegment(seg, 'coturnleft',   45, 2, xxx, 4);
seg = trjsegment(seg, 'uniform',      100);
seg = trjsegment(seg, 'coturnright',  10*5, 9, xxx, 4);
seg = trjsegment(seg, 'uniform',      100);
seg = trjsegment(seg, 'climb',        10, 2, xxx, 50);
seg = trjsegment(seg, 'uniform',      100);
seg = trjsegment(seg, 'descent',      10, 2, xxx, 50);
seg = trjsegment(seg, 'uniform',      100);
seg = trjsegment(seg, 'deaccelerate', 5,  xxx, 2);
seg = trjsegment(seg, 'uniform',      100);
trj = trjsimu(avp0, seg.wat, ts, 1);
insplot(trj.avp);
imuplot(trj.imu);

%%
inst = [0.3;0;0.5]*glv.deg; kod = 1;
trj1 = odsimu(trj, inst, kod);  % high-accuracy OD & AVP re-simu
binfile([glv.datapath,'imuod.bin'], [trj1.imu(:,1:6), trj1.od(:,1), trj1.avp]); % 17 column
avpd = drpure([trj1.imu(:,1:6), trj1.od], trj1.avp0, inst, kod);
avpcmpplot(trj1.avp, avpd, 'avp');

%%
% use VC60 simulation to get SINS/OD results

%%
ins = binfile([glv.datapath,'ins.bin'], 19);  insplot(ins(:,1:16),'vb'); % odpplot(ins(:,[17:19,16]));
avpcmpplot(trj1.avp, ins(:,[1:9,16]), 'avp');
psinstypedef(156);
[xk, pk, zk, rk, sk]=kffile([glv.datapath,'kf.bin'], 18, 3);
kfplot(xk, pk);
xpplot(xk, pk, 16:18, 1, 'kappa');
xpplot(zk,rk,1:3);
stateplot(sk,4);

