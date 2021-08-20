ggnss;
% load cubb1870
%% GPS
t1=151200; t2 = t1+7200;
prn = 112;
eph10 = eph(eph(:,ggps.ephs.Toc)==t1,:);     eph20 = eph(eph(:,ggps.ephs.Toc)==t2,:);
eph1 = eph10(eph10(:,ggps.ephs.PRN)==prn,:); eph2 = eph20(eph20(:,ggps.ephs.PRN)==prn,:);
T = t1:30:t2;
len = length(T); err = zeros(len,6);
for k=1:length(T)
    [satpv1, clkerr1] = satPosVelBatch(T(k), eph1);
    [satpv2, clkerr2] = satPosVelBatch(T(k), eph2);
    err(k,:) = satpv1-satpv2;
end
myfigure,
subplot(231),plot(T-T(1), err(:,1:3));grid on; xygo('\deltaP / m'); title('GPS');
subplot(234),plot(T-T(1), err(:,4:6)); grid on; xygo('\deltaV / m/s');
%% BD
t1=151200+14; t2 = t1+7200;
prn = 504;
eph10 = eph(eph(:,gbd.ephs.Toc)==t1,:);     eph20 = eph(eph(:,gbd.ephs.Toc)==t2,:);
eph1 = eph10(eph10(:,gbd.ephs.PRN)==prn,:); eph2 = eph20(eph20(:,gbd.ephs.PRN)==prn,:);
T = t1:30:t2;
len = length(T); err = zeros(len,6);
for k=1:length(T)
    [satpv1, clkerr1] = bdsatPosVelBatch(T(k), eph1);
    [satpv2, clkerr2] = bdsatPosVelBatch(T(k), eph2);
    err(k,:) = satpv1-satpv2;
end
subplot(232),plot(T-T(1), err(:,1:3));grid on; xygo('\deltaP / m'); title('BD');
subplot(235),plot(T-T(1), err(:,4:6)); grid on; xygo('\deltaV / m/s');
%% GLONASS
t1=105300; t2 = t1+1800;
prn = 203;
eph10 = eph(eph(:,gglo.ephs.Toc)==t1,:);     eph20 = eph(eph(:,gglo.ephs.Toc)==t2,:);
eph1 = eph10(eph10(:,gglo.ephs.PRN)==prn,:); eph2 = eph20(eph20(:,gglo.ephs.PRN)==prn,:);
T = t1:30:t2;
len = length(T); err = zeros(len,6);
for k=1:length(T)
    [satpv1, clkerr1] = glosatPosVel(T(k), eph1);
    [satpv2, clkerr2] = glosatPosVel(T(k), eph2);
    err(k,:) = satpv1-satpv2;
end
subplot(233),plot(T-T(1), err(:,1:3));grid on; xygo('\deltaP / m'); title('GLONASS');
subplot(236),plot(T-T(1), err(:,4:6)); grid on; xygo('\deltaV / m/s');