function [eb, att, yaw2] = imudpdrift(imu, t1, t2, pos, dyaw)
% IMU drift calculation using double-angular position, on static base.
%
% Prototype: eb = imudpdrift(imu, t1, t2, pos, dyaw)
% Inputs: imu - IMU data array
%         t1 - 0 pisition time interval [t1(1); t1(2)]
%         t2 - pi pisition time interval [t2(1); t2(2)]
%         pos - [lat,lon,hgt] or lat
%         dyaw - yaw diference between 0&pi position
% Output2: eb - gyro drift
%          att - attitudes for double-angular position
%          yaw2 - align yaw for pi-position
%
% Example
%     att0 = [1;0;10]*glv.deg;  pos0 = posset(34, 116, 480);
%     att = attrottt(att0, [1, 0,0,1, 180*glv.deg, 20,140,140], 0.01);
%     len = length(att);
%     avp = [att(:,1:3),zeros(len,3),repmat(pos0',len,1),att(:,4)];
%     imu = avp2imu(avp); % imuplot(imu);
%     imuerr = imuerrset([1;2;3]*0.1, 100, 0.0001, 1.0);
%     [eb,att] = imudpdrift(imuadderr(imu,imuerr), [1;100], [200;250], pos0);
%
% See also  sysclbt, clbtfile, imuadderr, imulvS, lsclbt.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 19/04/2022
global glv
    if nargin<5, dyaw=180*glv.deg; end
    if length(t1)<2; t1=[-inf,t1]; end 
    if length(t2)<2; t2=[t2,inf]; end
    ts = diff(imu(1:2,end));
    eth = earth(pos);  wN=eth.wnie(2); wU=eth.wnie(3);
    imu1 = datacut(imu, t1(1),t1(2));   wb1 = mean(imu1(:,1:3))'/ts;
	imu2 = datacut(imu, t2(1),t2(2));   wb2 = mean(imu2(:,1:3))'/ts;
    att(1,:) = alignsb(imu1, pos)';
    att(2,:) = alignsb(imu2, pos);    yaw2=att(2,3); att(2,3)=att(1,3)+dyaw;
    C1 = a2mat(att(1,:)'); C2 = a2mat(att(2,:)');
    wb10 = C1'*eth.wnie;   wb20 = C2'*eth.wnie;
	eb = (wb1-wb10+wb2-wb20)/2;
%     dw = wb2-wb1; dC = (C2-C1)';
%     phiU = (dw(1)-dC(1,2)*wN-dC(1,3)*wU)/(dC(1,1)*wN);
%     eb = ( wb1+wb2 - (C1+C2)'*(eye(3)-askew([0;0;phiU]))*eth.wnie )/2;  % "惯性仪器测试与数据分析"式（10.4-33）
    imumeanplot([imu1;imu2], fix(1/ts), glv.dph);
    subplot(323), title(sprintf('yaw1= %.4f; yaw2= %.4f / %.4f (\\circ)', att(1,3)/glv.deg,att(2,3)/glv.deg,yaw2/glv.deg));
    subplot(325), title(['eb=[ ',sprintf('%.4f ', eb/glv.dph),'](\circ)/h']);
    