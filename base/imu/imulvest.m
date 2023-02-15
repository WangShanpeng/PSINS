function [lxyz, av] = imulvest(imu, pos, rott, statict)
% SIMU inner lever estimation.
%
% Prototype: [lxyz, av] = imulvest(imu, pos, rott)
% Inputs: imu - SIMU data
%         pos - geographical position = [latitude; longitude; height]
%         rott - rotation time sequence [ rx-begin rx-end; ry-begin ryend; rz-begin rz-end]
%         statict - static time before rotation 
% Output: lxyz - SIMU inner parameter
%         av - navigation attitude & velocity
%
% Example:
% glvs;
% ts = 0.01;
% att0 = [0,0,0; 90,0,0; 0,-90,0]*glv.deg;  rxyz = [3,2,1];
% imu = [];
% for k=1:3,
%     paras = [ 1  0,0,0, 180*glv.deg, 10, 40, 10 ]; paras(rxyz(k)+1)=1;
%     att = attrottt(att0(k,:)', paras, ts);
%     imu = [imu; avp2imu(att, pos0)];
% end
% imu = addclmt(imu(:,1:6),ts); % imuplot(imu);
% clbt.rx = -[10;20;30]/1000; clbt.ry = -[40;50;60]/1000; clbt.rz = -[0;0;0]/1000;
% imu1 = imuclbt(imu, clbt);
% rott = [ 35,55; [35,55]+60; [35,55]+120 ]; 
% [lxyz, av] = imulvest(imu1, pos0, rott);  insplot(av);
%
% See also  imulvplot, imulever, sysclbt.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/11/2022
global glv
    if nargin<4, statict = 30; end
    imu = imu(:,[1:6,end]);
    [nn,ts,nts] = nnts(2, diff(imu(1:2,end)));
    dotwf = imudot(imu, 5.0);  Cba = eye(3);
    wnie = glv.wie*[0;cos(pos(1));sin(pos(1))]; gn = [0;0;-imu2g(imu,0,statict)];
    qnb = a2qua(alignsb(datacut(imu,1,statict),pos)); vn = zeros(3,1);
    len = length(imu);
    av = zeros(fix(len/nn),7);  kk = 1;
    timebar(nn, len, 'SIMU inner lever estimation.');
    stage=zeros(3,1);  H1 = zeros(3,9);  H2 = zeros(3,9);  H3 = zeros(3,9); 
    for k=1:nn:len-nn
        k1 = k+nn-1;
        wm = imu(k:k1,1:3); vm = imu(k:k1,4:6); t = imu(k1,end); dwb = mean(dotwf(k:k1,1:3),1)';
        [phim, dvbm] = cnscl([wm,vm]);
        wb = phim/nts; fb = dvbm/nts;
        SS = lvS(Cba, wb, dwb); % fL = SS*[rx;ry;rz];  % lever arm
        fn = qmulv(qnb, fb); % - Cnb*fL;
        an = rotv(-wnie*nts/2, fn) + gn;
        vn = vn + an*nts;  % vel update
        Cnb = q2mat(qnb);
        if     t>rott(1,1) && t<rott(1,2), H1 = H1 + Cnb*SS*nts;
        elseif t>rott(2,1) && t<rott(2,2), H2 = H2 + Cnb*SS*nts;
        elseif t>rott(3,1) && t<rott(3,2), H3 = H3 + Cnb*SS*nts;  
        else vn=zeros(3,1);  end
        qnb = qupdt2(qnb, phim, wnie*nts);  % att update
        av(kk,:) = [q2att(qnb); vn; t];  kk=kk+1;
        timebar(nn);
        for s=1:3
            if t>rott(s,1)-0.1 && stage(s)==0, stage(s)=1; qnb=a2qua(alignsb(datacut(imu,rott(s,1)-statict,rott(s,1)),pos)); end
        end
    end
    av = no0(av,1:3);
    dvn = [];
    for k=1:3
        avi1 = mean(datacut(av,rott(k,1), rott(k,1)+1))'; avi2 = mean(datacut(av,rott(k,2)-1, rott(k,2)))'; 
        dvn = [dvn; avi2(4:6)-avi1(4:6)];
    end
    H = [H1; H2; H3];
    lxy = lscov(H(:,1:6), dvn);    lxz = lscov(H(:,[1:3,7:9]), dvn);    lyz = lscov(H(:,4:9), dvn);  % dvn = Cnb*SS * [rx;ry;rz]
    lxyz = [[lxy; zeros(3,1)], [lxz(1:3); zeros(3,1); lxz(4:6)], [zeros(3,1); lyz]];
    myfig, 
    for k=1:3, avi=datacut(av,rott(k,1),rott(k,2)); subplot(2,2,k), plot(avi(:,end), avi(:,4:6)); xygo('V'); end
    lv=lxyz(:,1)*1000;
    subplot(2,2,4); plot(lv(1),lv(2),'>',lv(4),lv(5),'^',lv(7),lv(8),'*'); axis equal;
    xygo('x / mm', 'y / mm'); title('Inner level arm parameters');
    legend(sprintf('AXz: %4.1f',lv(3)),sprintf('AYz: %4.1f',lv(6)),sprintf('AZz: %4.1f',lv(9)));
    
function SS = lvS(Cba, wb, dotwb)
    U = (Cba')^-1; V1 = Cba(:,1)'; V2 = Cba(:,2)'; V3 = Cba(:,3)';
    Q11 = U(1,1)*V1; Q12 = U(1,2)*V2; Q13 = U(1,3)*V3;
    Q21 = U(2,1)*V1; Q22 = U(2,2)*V2; Q23 = U(2,3)*V3;
    Q31 = U(3,1)*V1; Q32 = U(3,2)*V2; Q33 = U(3,3)*V3;
    W = askew(dotwb)+askew(wb)^2;
    SS = [Q11*W, Q12*W, Q13*W; Q21*W, Q22*W, Q23*W; Q31*W, Q32*W, Q33*W];    