function [att1, dKgz, db] = alignsbtp(imu, pos, Y1Y2, T1T2, isfig)
% SINS align on static base using two-position method.
%
% Prototype: [att, dKgz, db] = alignsbtp(imu, pos, Y1Y2, T1T2, isfig)
% Inputs: imu - SIMU data
%         pos - initial position
%         Y1Y2 - yaw1/yaw2 reference for first/second angular position, 0 for no reference
%         T1T2 - [-inf,T1T2(1)]/[T1T2(2),inf] for first/second angular position
%         isfig - figure flag
% Outputs: att1, attitude align results Euler angles in first angular position
%          dKgz, db - z to x/r axis gyro misalignment angles, acc bias
%
% Example:
%    imuerr = imuerrset(0.0,100.,0,0, 0,0,0,0, 00,0,0,0);
%    [imu1, att0] = imustatictp([[1;2;10]*glv.deg; glv.pos0], 0.1, 300, 10*glv.dps, 180*glv.deg, imuerr);
%    [att, dKgz, db] = alignsbtp(imu1, glv.pos0, [10,190]*glv.deg, [130,170]);
%
% See also  alignsb, alignvn, aa2dkg, alignvntp, alignsbtp1.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 11/11/2021
global glv
    if nargin<5, isfig=1; end
    ts = diff(imu(1:2,end));
    w = normv(imu(:,1:3))/ts;
    if nargin<4,
        idx(1) = find(w>1*glv.dps, 1, 'first'); idx(1)=idx(1)-fix(5/ts);
        idx(2) = find(w>1*glv.dps, 1, 'last'); idx(2)=idx(2)+fix(5/ts);
        T1T2 = imu(idx,end);
    end
    if length(pos)==1,  pos = [pos; 0; 0];   end
    att1 = alignsb(datacut(imu,-inf,T1T2(1)), pos, 0);
    att2 = alignsb(datacut(imu,T1T2(2),inf), pos, 0);
    if nargin<3, Y1Y2=0; end
    if length(Y1Y2)>1, att1(3)=Y1Y2(1); att2(3)=Y1Y2(2); end
    avp = inspure(imu, [att1; pos], 'v', 0);  % insplot(avp);
    att = avp(:,[1:3,end]);
    if isfig==1
        myfig;
        a1 = datacut(att,imu(1,end),T1T2(1)); a2 = datacut(att,T1T2(1),T1T2(2)); a3 = datacut(att,T1T2(2),imu(end,end));
        subplot(331), plot(a1(:,end), a1(:,1)/glv.deg, [imu(1,end),T1T2(1)]', repmat(att1(1),2,1)/glv.deg); xygo('p');  legend('Att Track', 'Align Static');
        subplot(334), plot(a1(:,end), a1(:,2)/glv.deg, [imu(1,end),T1T2(1)]', repmat(att1(2),2,1)/glv.deg); xygo('r');
        subplot(337), plot(a1(:,end), a1(:,3)/glv.deg, [imu(1,end),T1T2(1)]', repmat(att1(3),2,1)/glv.deg); xygo('y');
        subplot(332), plot(a2(:,end), a2(:,1)/glv.deg); xygo('p');  title('T1 -> T2');
        subplot(335), plot(a2(:,end), a2(:,2)/glv.deg); xygo('r');
        subplot(338), plot(a2(:,end), a2(:,3)/glv.deg); xygo('y');
        subplot(333), plot(a3(:,end), a3(:,1)/glv.deg, [T1T2(2),imu(end,end)]', repmat(att2(1),2,1)/glv.deg); xygo('p');
        subplot(336), plot(a3(:,end), a3(:,2)/glv.deg, [T1T2(2),imu(end,end)]', repmat(att2(2),2,1)/glv.deg); xygo('r');
        subplot(339), plot(a3(:,end), a3(:,3)/glv.deg, [T1T2(2),imu(end,end)]', repmat(att2(3),2,1)/glv.deg); xygo('y');
%         insplot(att,'a');
%         subplot(211), plot([imu(1,end),T1T2(1)]', repmat(att1(1:2)',2,1)/glv.deg, '*-.m',  [T1T2(2),imu(end,end),]', repmat(att2(1:2)',2,1)/glv.deg, '*-.m');
%         subplot(212), plot([imu(1,end),T1T2(1)]', repmat(att1(3)',2,1)/glv.deg, '*-.m',  [T1T2(2),imu(end,end),]', repmat(att2(3)',2,1)/glv.deg, '*-.m');
    end
    att20 = mean(datacut(att,T1T2(2),inf))';
    if length(Y1Y2)<2, att2(3)=att20(3); end  % mu x/y hold for small yaw error
    mu = aa2mu(att2, att20(1:3));
    dKgz = [mu(2); -mu(1); 0]/2;  % x/y in rad, z in ppm
    if length(Y1Y2)>1, dKgz(3)=-mu(3)/sum(imu(:,3)); end
    db = [-mu(2); mu(1); 0]/2*glv.g0;
    if isfig==1
        subplot(333), title(sprintf('dKgz: %.1f, %.1f arcsec, %.1f ppm OR db: %.1f, %.1f ug', ...
            dKgz(1)/glv.sec,dKgz(2)/glv.sec,dKgz(3)/glv.ppm, db(1)/glv.ug,db(2)/glv.ug));
    end
