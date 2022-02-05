% SINS/CNS integrated navigation simulation.
% See also  test_SINS_CNS_184.
% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 25/11/2021
glvs;
%% to VC60: Demo_SINSCNS
trj = trjfile('trj10ms.mat');
[qis, utc0] = cnssimu(trj.avp, [10;10;30]*glv.sec, [10;10;30]*glv.min, [2021;11;22;12*3600; -0.1;37]);
imucns = combinedata(trj.imu, qis(1:100:end,:));
binfile('imucns.bin', imucns);
%%
% use VC60 simulation to get SINS/CNS results ...

%% from VC60: Demo_SINSCNS
avp = binfile('ins.bin', 16);  insplot(avp);
avpcmpplot(trj.avp, avp(:,[1:9,end]));
psinstypedef(156);
[xk, pk, zk, rk, sk] = kffile('kf.bin', 18, 9);
kfplot(xk,pk,1:15);
xpplot(xk,pk,16:18, glv.min, 'mu');
xpplot(zk,rk,7:9, glv.sec, 'zk');
stateplot(sk);

