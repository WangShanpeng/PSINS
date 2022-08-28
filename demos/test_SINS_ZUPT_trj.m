% Trajectory generation for later simulation use.
% See also  test_SINS_ZUPT.
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/02/2022
glvs
ts = 0.1;       % sampling interval
avp0 = [[0;0;0]; [0;0;0]; glv.pos0]; % init avp
% trajectory segment setting
xxx = [];
seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'uniform',      50);
seg = trjsegment(seg, 'accelerate',   10, xxx, 1);
seg = trjsegment(seg, 'uniform',      50);
seg = trjsegment(seg, 'coturnleft',   45, 2, xxx, 4);
seg = trjsegment(seg, 'uniform',      50);
seg = trjsegment(seg, 'coturnright',  10, 9, xxx, 4);
seg = trjsegment(seg, 'uniform',      50);
seg = trjsegment(seg, 'climb',        10, 2, xxx, 50);
seg = trjsegment(seg, 'uniform',      50);
seg = trjsegment(seg, 'descent',      10, 2, xxx, 50);
seg = trjsegment(seg, 'uniform',      40);
seg = trjsegment(seg, 'deaccelerate', 5,  xxx, 2);
seg = trjsegment(seg, 'uniform',      600-sum(seg.wat(:,1)));
% generate, save & plot
trj = trjsimu(avp0, seg.wat, ts, 6);
trjfile('trj10ms.mat', trj);
insplot(trj.avp);
imuplot(trj.imu);

