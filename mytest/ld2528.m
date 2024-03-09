function [imu, od, gps, gpsyaw, avpr, t0] = ld2528(fname, clm, t0)
% typedef struct {
% 	double t;
% 	CVect3 wm, vm;
% 	double od;
% 	CVect3 posgps; double dop; CVect3 vngps; double yawgps;
% 	CVect3 pos610, att610, vn610;
% } FileRecord;
if clm==25
    dd = binfile(fname, 25);
    if nargin<3, t0 = dd(1,1); end
    t = dd(:,1)-t0;
    imu = [dd(:,2:4),dd(:,5:7),t];
    od = [dd(:,8), t];
    gps = [dd(:,13:15), dd(:,9:10), dd(:,11), t];  gps = gps(gps(:,4)>0.1,:);
    gpsyaw = no0([dd(:,16), t], 1);
    avpr = no0([dd(:,[21,20,22:25,17:19]),t],1);
elseif clm==28
    dd = binfile(fname, 28);
    if nargin<3, t0 = dd(1,1); end
    tt = dd(:,1)-t0;
    imu = [[dd(:,2:4),dd(:,5:7)],tt];  ts=mean(diff(imu(:,end)));
    od = [dd(:,8)*ts,tt];
    gps = no0([dd(:,13:15),dd(:,9:10),dd(:,11),tt],4);
    gpsyaw = no0([dd(:,18), tt], 1);
    avpr = no0([avpidx(dd, [24,23,25, 26,27,28, 20:21,22]),tt],1); 
elseif clm==32
    dd = binfile(fname, 32);
    if nargin<3, t0 = dd(1,1); end
    tt = dd(:,1)-t0;
    imu = [[dd(:,2:4),dd(:,5:7)],tt];  ts=mean(diff(imu(:,end)));
    od = [dd(:,8)*ts,tt];
    gps = no0([dd(:,13:15),dd(:,9:10),dd(:,11),tt],4);
    gpsyaw = no0([dd(:,18), tt], 1);
    avpr = no0([avpidx(dd, [24,23,25, 26,27,28, 20:21,22]),tt],1); 
elseif clm==38
    dd = binfile(fname, 38);
    if nargin<3, t0 = dd(1,1); end
    tt = dd(:,1)-t0;
    imu = [[dd(:,2:4),dd(:,5:7)],tt];  ts=mean(diff(imu(:,end)));
    od = [dd(:,[8,33:36])*ts,tt];  % mean(BL,BR), FL, FR, BL, BR
    gps = no0([dd(:,13:15),dd(:,9:10),dd(:,11),tt],4);
    gpsyaw = no0([dd(:,18), tt], 1);
    avpr = no0([avpidx(dd, [24,23,25, 26,27,28, 20:21,22]),tt],1); 
end
