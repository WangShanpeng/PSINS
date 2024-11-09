function [imu, gps, avp, edb, gpsd] = olload(fname)
%1  %check %na0 %na1 
%4  %Gx %Gy %Gz %Ax %Ay %Az %Temp %t 
%12 %gpsVE %gpsVN %gpsVU %gpsLat1 %gpsLon1 %gpsHgt %gpsStatus %gpsDelay 
%20 %Pitch %Roll %Yaw %VE %VN %VU %Lat %Lon %H %29 ebx %eby %ebz %dbx %dby %dbz 
dd = load(fname);  dd=dd(dd(:,11)>1e-6,:);
imu = no0(dd(:,4:11)); % imuplot(imu); % ttest(imu);
gps = no0(dd(:,[12:18,11])); % gpsplot(gps);   % ttest(gps);
avp = no0(dd(:,[20:28,11])); % insplot(avp);  % ttest(avp);
edb = no0(dd(:,[29:34,11]));
gpsd = no0(dd(:,[19,11]));