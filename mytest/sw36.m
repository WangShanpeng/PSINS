function [imu, avp, gps, xyz, t0, dd] = sw36(fname, t0, tobin)
% 	double seq, time, exp_time, enu_x, enu_y, enu_z, enu_rx, enu_ry, enu_rz, enu_vx, enu_vy, enu_vz,
% 		fix_type, acc_x, acc_y, acc_z, g_x, g_y, g_z, lon, lat, alt, speed, heading, gps_lon, gps_lat, gps_alt, sat_htop, ntp,
% 		err_x, err_y, err_z, std_x, std_y, std_z, err_angle;
global glv
    if strcmp(fname(end-2:end),'bin')==1
        dd = binfile(fname, tobin);   % sw36(fname, t0, clm)
    else
        dd = load(fname);
    end
    if nargin<3, tobin=0; end
    if nargin<2, t0=dd(1,2); end
    if t0==-inf, t0=dd(1,2); end
    dd(:,2) = dd(:,2)-t0;  % ttest(dd(:,2))
    if tobin==1, binfile(fnameext(fname,'bin'),dd); end  % sw28(fname, t0, 1);
    if size(dd,2)==15 
        %seq, time, exp_time, enu_x, enu_y, enu_z, enu_rx, enu_ry, enu_rz, enu_vx, enu_vy, enu_vz, lon, lat, alt
        imu = 1;
        avp = [dd(:,7:9)*glv.deg, dd(:,10:12), dd(:,[14,13])*glv.deg, dd(:,15), dd(:,2)];
        return;
    end
    idx = dd(:,27)>0;
    gps = [vy2vn(dd(idx,23),-dd(idx,24)*glv.deg), dd(idx,[26,25])*glv.deg,dd(idx,[27,28,2])]; %  gpsplot(gps); gpssatplot(gps(:,end-1:end));
    avp = no0([-dd(:, 7:9)*glv.deg, dd(:, 10:12), dd(:,[21,20])*glv.deg, dd(:,[22,2])],1:3); % insplot(avp);
    avp(:,3) = yawcvt(avp(:,3)-pi,'c360cc180');
    imu = [dd(:,[17:19,14:16])*0.01, dd(:,2)]; % imuplot(imu);
    xyz = dd(:, [4:6,2]);  % miniplot(xyz, 'xyz');
