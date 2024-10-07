function [att, eb, db] = alignpetp(imu, pos, T1T2, eU, isfig)
% SINS align on static base using two-position method.
%
% Prototype: [att, eb, db] = alignpetp(imu, pos, T1T2, eU, isfig)
% Inputs: imu - SIMU data
%         pos - initial position
%         T1T2 - [-inf,T1T2(1)]/[T1T2(2),inf] for first/second angular position
%                if exist T1T2(3), for leveling time
%         eU - upward gyro drift
%         isfig - figure flag
% Outputs: att, attitude align results Euler angles
%          eb, db - gyro drift, acc bias
%
% Example:
%    imuerr = imuerrset(0.1,1000.,0,0, 0,0,0,0, 00,0,0,0);
%    [imu, att0] = imustatictp([0*[.1;.2;.3]*glv.deg; glv.pos0], 0.1, 600, 10*glv.dps, 180*glv.deg, imuerr); % insplot(att0, 'a');
%    [att, eb, db] = alignpetp(imu, glv.pos0, [280,320, 30], 0.1*glv.dph);
%    insplot(att0, 'a');  subplot(211), plot(att0(end,end), att(1:2)/glv.deg, 'mo'), subplot(212), plot(att0(end,end), att(3)/glv.deg, 'mo');
%    imu1 = imuclbt(imu, eb, db);  % imuplot(imu1);
%    [att1, eb1, db1] = alignpetp(imu1, glv.pos0, [280,320, 30], 0.0*glv.dph);  % repeat
%    imu2 = imuclbt(imu1, eb1, db1);  % imuplot(imu2);
%    [att2, eb2, db2] = alignpetp(imu2, glv.pos0, [280,320, 30], 0.0*glv.dph);
%
% See also  alignsb, alignvn, imustatictp, alignvntp, alignsbtp1.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 11/11/2021
global glv
    if nargin<5, isfig=1; end
    if nargin<4, eU=0; end
    ts = diff(imu(1:2,end));
    if length(pos)==1,  pos = [pos; 0; 0];   end
    i1 = datacut(imu,-inf,T1T2(1),'(]');
    ii = datacut(imu,T1T2(1),T1T2(2),'(]');
    i2 = datacut(imu, T1T2(2),inf,'[)');
    if isfig==2, isfig=1;  % TIPS: for constant leveling acc simulation
        i1(:,4)=mean(i1(:,4));  i1(:,5)=mean(i1(:,5));
        i2(:,4)=mean(i2(:,4));  i2(:,5)=mean(i2(:,5));
    end
    [att1, att10, eN1, dU1, av1] = alignpe(i1, pos, eU, 30, 1);
    avp = inspure(ii, [av1(end,1:3)'; pos], 'v', 0);  % insplot(avp);
    [att2, att20, eN2, dU2, av2] = alignpe(i2, pos, eU, 30, 1);
    if avp(end,end)<av2(1,end) % to last IMU record
        idx1 = find(avp(end,end)<=imu(:,end),1,'first');
        idx2 = find(av2(1,end)<=imu(:,end),1,'first');
        qnb = qupdt(a2qua(avp(end,1:3)'), sum(imu(idx1+1:idx2, 1:3),1)');
        avp(end+1,[1:3,end]) = [q2att(qnb); imu(idx2,end)]';
    end
    phi = aa2phi(avp(end,1:3)', av2(1,1:3)')/2;
    att = aaddphi(av2(end,1:3)', phi);
    eE = -phi(3)*cos(pos(1))*glv.wie;
    eN = (eN1-eN2)/2;
    dE = phi(2)*glv.g0;
    dN = -phi(1)*glv.g0;
    Cnb = a2mat(av1(1,1:3)');
    eb = Cnb'*[eE;eN;eU];  db = Cnb'*[dE;dN;dU2];
    if length(T1T2)>2  % the last T1T2(3) sec, leveling
        if T1T2(3)<5, return; end
        [a2, idx] = getat(av2, imu(end,end)-T1T2(3));
        i1 = find(av2(idx,end)<imu(:,end),1,'first'); % [av2(idx,end), imu(i1,end)]
        [att0, attk] = aligncmps(imu(i1:end,:), a2qua(a2(1:3)), pos, [T1T2(3)/3;T1T2(3)], [100;100], 1);
        att = [att0(1:2);att(3)];
    end
    if isfig==1
        myfig;
        subplot(331), plot(av1(:,end), av1(:,1)/glv.deg,'-', avp(1,end), avp(1,1)/glv.deg,'om'); xygo('p');  % stage 1
        subplot(334), plot(av1(:,end), av1(:,2)/glv.deg,'-', avp(1,end), avp(1,2)/glv.deg,'om'); xygo('r');
        subplot(337), plot(av1(:,end), av1(:,3)/glv.deg,'-', avp(1,end), avp(1,3)/glv.deg,'om'); xygo('y');
        subplot(333), plot(av2(:,end), av2(:,1)/glv.deg,'-', avp(end,end), avp(end,1)/glv.deg,'om'); xygo('p');  % stage 2
        subplot(336), plot(av2(:,end), av2(:,2)/glv.deg,'-', avp(end,end), avp(end,2)/glv.deg,'om'); xygo('r');
        subplot(339), plot(av2(:,end), av2(:,3)/glv.deg,'-', avp(end,end), avp(end,3)/glv.deg,'om'); xygo('y');
        subplot(332), plot(avp(:,end), avp(:,1)/glv.deg); xygo('p');  title('T1 -> T2');  % rotate
        subplot(335), plot(avp(:,end), avp(:,2)/glv.deg); xygo('r');
        subplot(338), plot(avp(:,end), avp(:,3)/glv.deg); xygo('y');
        subplot(333), title([sprintf('%.4f, ', eb/glv.dph),'(dph); ', sprintf('%.1f, ', db/glv.ug),'(ug)']);
        subplot(339), title([sprintf('%.4f, ', att/glv.deg),'(deg)']);
    end

%     if length(T1T2)>2  % the last T1T2(3) sec, dVE\N\U to phiE\N&dU
%         if T1T2(3)<5, return; end
%         [a2, idx] = getat(av2, imu(end,end)-T1T2(3));
%         i1 = find(av2(idx,end)<imu(:,end),1,'first'); % [av2(idx,end), imu(i1,end)]
%         av = inspure(imuclbt(imu(i1:end,:),eye(3),eb,eye(3),db), [aaddphi(a2(1:3),phi); pos], 'f', 1);  close(gcf);
%         px1 = polyfit(av(:,end),av(:,5),1); 
%         py1 = polyfit(av(:,end),av(:,4),1); 
%         pz1 = polyfit(av(:,end),av(:,6),1); 
%         phi1 = [-px1(end-1); py1(end-1); 0]/glv.g0;  dU1=pz1(end-1); % phi1/glv.sec
% %         av1 = inspure(imuclbt(imu(i1:end,:),eye(3),eb,eye(3),Cnb'*[dE;dN;dU+dU1]), [aaddphi(a2(1:3),phi+phi1); pos], 'f');
%         att = aaddphi(att, phi1);  db = Cnb'*[dE;dN;dU+dU1];
%     end

