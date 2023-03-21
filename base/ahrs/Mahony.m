function attebfn = Mahony(imu, tau, att0)
% See also  MahonyInit, MahonyUpdate, inspure.
global glv
    if ~exist('att0', 'var'), att0 = zeros(3,1); end
    if ~exist('tau', 'var'), tau = 2; end
    [nn,ts,nts] = nnts(1,diff(imu(1:2,end)));
    ahrs = MahonyInit(tau, att0);
    attebfn = zeros(length(imu)/nn,10);
    ki = timebar(1, length(imu)/nn, 'Mahony processing.');
    for k=1:nn:length(imu)-nn+1
        [phim, dvbm] = cnscl(imu(k:k+nn-1,1:6));
        ahrs = MahonyUpdate(ahrs, [phim;dvbm]', 0, nts);
%         attebfn(ki,:) = [m2att(ahrs.Cnb); ahrs.exyzInt; ahrs.Cnb*dvbm/nts+[0;0;-glv.g0]; imu(k+nn-1,end)]';
        attebfn(ki,:) = [m2att(ahrs.Cnb); ahrs.Cnb*phim/nts; ahrs.Cnb*dvbm/nts+[0;0;-glv.g0]; imu(k+nn-1,end)]';
        ki = timebar;
    end
    attebfn(ki:end,:) = [];  t = attebfn(:,end);
	myfig;
    for k=1:3, attebfn(:,k) = angle2c(attebfn(:,k));  end
	subplot(221), plot(t, attebfn(:,1:2)/glv.deg), xygo('pr');
	subplot(223), plot(t, attebfn(:,3)/glv.deg), xygo('y');
% 	subplot(222), plot(t, attebfn(:,4:6)/glv.dph), xygo('eb');
	subplot(222), plot(t, attebfn(:,4:6)/glv.dps), xygo('wb / \circ/s');
	subplot(224), plot(t, attebfn(:,7:9)/glv.g0), xygo('fn / g');
    