glvs;
ts = 0.01;
fid = fopen('PSINS20210803_090711.bin', 'rb'); dd = fread(fid, [35, inf], 'float32')'; fclose(fid);
imu = [[dd(:,3:5)*glv.dps, dd(:,6:8)]*ts, dd(:,2)]; imuplot(imu);

imu(:,1:3)=delbias(imu(:,1:3),[-4,1.3,0]'*glv.dps*ts);
atteb = Mahony(imu, 10);

ae = binfile('mahony.bin', 7);
myfig; plot(ae(:,end), ae(:,1:2)/glv.deg); xygo('pr');
hold on; plot(atteb(:,end)-atteb(1,end), atteb(:,1:2)/glv.deg, 'm');

myfig; plot(ae(:,end), ae(:,4:5)/glv.dps); 
hold on; plot(atteb(:,end)-atteb(1,end), atteb(:,4:5)/glv.dps, 'm');  grid on