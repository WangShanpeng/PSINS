function [imu, sf, Kga, Ka] = imuscale(imu, t, w, f)
% Scale gyro(acc) to given angular rate(force) at time t, such that
% [w; f]'*ts = imu(k,1:6) .* sf' .
%
% Prototype: imu = imuscale(imu, t, w, f)
% Inputs: imu - SIMU data array
%         t - specific times =[tgx,tgy,tgz, tax,tay,taz]
%         w/f - given angular rate(force), in rad/s (m/s^2)
% Outputs: imu - SIMU data array after scaling
%          sf - scale factor array
%          Kga - coarse installation matrix, or Kg & Ka
%
% See also  sysclbt, imuinc, imudot, imurot, imustaticscale.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 18/01/2018, 20/01/2023
    if nargin<4, f=[1;1;1]*9.8; end
    if length(w)==1, w=[w;w;w]; end
    if length(f)==1, f=[f;f;f]; end
    wf = [w; f];  sf = wf;
    ts = diff(imu(1:2,end));  dt = 50;
    if size(imu,2)==11  % for 5-axis Redundant IMU, X/Y/Z/S/T
        [imu1, sf, Kg, Ka] = imuscale(imu(:,[1:3,6:8,end]), t, w, f);  % orthogonal X/Y/Z first
        wST = zeros(3,2); fST = wST;  wsfST = zeros(1,2); fsfST = wsfST;
        for k=1:3,
            idx = find(imu1(:,end)>t(k),1,'first');   wST(k,:) = mean(imu(idx-dt:idx+dt,4:5))/ts;  wsfST = wsfST+(wST(k,:)/w(k)).^2;
            idx = find(imu1(:,end)>t(k+3),1,'first'); fST(k,:) = mean(imu(idx-dt:idx+dt,9:10))/ts; fsfST = fsfST+(fST(k,:)/f(k)).^2;
        end
        wsfST = sqrt(1./wsfST);  fsfST = sqrt(1./fsfST);  % ST scale factor
        sf = [sf(1:3); wsfST'; sf(4:6); fsfST'];
        imu = [imu1(:,1:3), imu(:,4)*wsfST(1), imu(:,5)*wsfST(2), ...
               imu1(:,4:6), imu(:,9)*fsfST(1), imu(:,10)*fsfST(2), imu1(:,end)];
        Kg(:,4)=wST(:,1)*wsfST(1)./w; Kg(:,5)=wST(:,2)*wsfST(2)./w;  Kga=Kg; % ST installation cos(ang)  Cbg
        Ka(:,4)=fST(:,1)*fsfST(1)./f; Ka(:,5)=fST(:,2)*fsfST(2)./f; 
        return;
    end  %%%%%%%%%%%%
    for k=1:6
        idx = find(imu(:,end)>t(k),1,'first');
        wf0 = mean(imu(idx-dt:idx+dt,:))/ts;
        sf(k) = wf(k)/wf0(k);
        imu(:,k) = imu(:,k)*sf(k);
    end
    Kga = zeros(6,3);
    for k=1:6
        idx = find(imu(:,end)>t(k),1,'first');
        wf0 = mean(imu(idx-dt:idx+dt,:))/ts;
        if k<=3, Kga(k,:) = wf0(1:3)/wf(k);
        else,    Kga(k,:) = wf0(4:6)/wf(k);  end
    end
    if nargout==4, Ka=Kga(4:6,:); Kga=Kga(1:3,:); end
    