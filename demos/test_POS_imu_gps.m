% POS and data fusion. A 34-state Kalman filter is used, 
% including: phi(3), dvn(3), dpos(3), eb(3), db(3), lever(3), dT(1), 
%            dKg(9), dKa(6). (total states 6*3+1+9+6=34)
% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 10/03/2021
glvs
psinstypedef(346);
load mimu_gps_POS.mat;  % download ?
imuplot(mimu);
gpsplot(dgps);
insplot(avpref);
ts = diff(mimu(1:2,end));
avp0 = avpadderr(avpref(1,1:9)', [1,1,3]'*glv.deg, 0);
ins = insinit(avp0,diff(mimu(1:2,end)));
kf = kfsetting(1, ts);
[ps,psf] = POSProcessing(kf, ins, mimu, dgps, 'avped', 'avp');
POSplot(psf);
avpcmpplot(avpref(:,[1:3,end]), psf.rf(:,[1:3,end]), 'a', 'phi');
avpcmpplot(avpref(:,[4:6,end]), setclm(psf.rf(:,[4:6,end]),3), 'v');
avpcmpplot(avpref(:,[7:9,end]), psf.rf(:,[7:9,end]), 'p');

