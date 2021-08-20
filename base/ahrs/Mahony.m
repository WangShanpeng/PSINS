function atteb = Mahony(imu, tau, att0)
% See also  MahonyInit, MahonyUpdate, inspure.
global glv
    if ~exist('att0', 'var'), att0 = zeros(3,1); end
    if ~exist('tau', 'var'), tau = 2; end
    [nn,ts,nts] = nnts(1,diff(imu(1:2,end)));
    ahrs = MahonyInit(tau, att0);
    atteb = imu*0;
    ki = timebar(1, length(imu)/2, 'Mahony processing.');
    for k=1:nn:length(imu)-nn+1
        [phim, dvbm] = cnscl(imu(k:k+nn-1,1:6));
        ahrs = MahonyUpdate(ahrs, [phim;dvbm]', 0, nts);
        atteb(ki,:) = [m2att(ahrs.Cnb);ahrs.exyzInt;imu(k+nn-1,end)]';
        ki = timebar;
    end
    atteb(ki:end,:) = [];  t = atteb(:,end);
	myfig;
    atteb(:,3) = angle2c(atteb(:,3));
	subplot(311), plot(t, atteb(:,1:2)/glv.deg), xygo('pr');
	subplot(312), plot(t, atteb(:,3)/glv.deg), xygo('y');
	subplot(313), plot(t, atteb(:,4:6)/glv.dph), xygo('eb');
    