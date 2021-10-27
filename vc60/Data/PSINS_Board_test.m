glvs;
ts = 0.01;
[imu, mag, bar, avp, gps, gs, temp, head, chksum] = psinsboardbin(0);
mhony = binfile('mahony.bin', 7);
myfig; plot(mhony(:,end), mhony(:,1:2)/glv.deg); xygo('pr');
hold on; plot(avp(:,end), avp(:,1:2)/glv.deg, 'm');

imu(:,1:3)=delbias(imu(:,1:3),[-4,1.3,0]'*glv.dps*ts);
atteb = Mahony(imu, 10);

mhony = binfile('mahony.bin', 7);
myfig; plot(mhony(:,end), mhony(:,1:2)/glv.deg); xygo('pr');
hold on; plot(avp(:,end), avp(:,1:2)/glv.deg, 'm');
hold on; plot(atteb(:,end)-atteb(1,end), atteb(:,1:2)/glv.deg, 'm');

myfig; plot(mhony(:,end), mhony(:,4:5)/glv.dps);  xygo('eb');
hold on; plot(atteb(:,end)-atteb(1,end), atteb(:,4:5)/glv.dps, 'm');  grid on