% SINS right-SE2(3)-error-model propagation accuracy verification.
% See also  test_SINS_error_model_verify, test_SINS_trj.
% Thanks to the help from Hongqiong Tang@NUE
% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 20/05/2023
glvs;
weie = [0;0;glv.wie];
trj = trjfile('trj100ms.mat');
[nn, ts, nts] = nnts(2, trj.ts);
%% error/init-state setting
imuerr = imuerrset(0.0, 000, 0.0, 0);  imu = imuadderr(trj.imu, imuerr);
phi = .01*[1;2;1.800]*glv.deg;  dve = 0*[1; 2; 3];  dpe = 0*[10; 20; 30];
[pe, Cen] = blh2xyz(trj.avp0(7:9));  qeb = m2qua(Cen*a2mat(trj.avp0(1:3)));  ve = Cen*trj.avp0(4:6)+cross(weie,pe);  ve0=ve; Ceb=q2mat(qeb);
qeb = qmul(rv2q(-phi),qeb);  Ceb=q2mat(qeb); Cee=rv2m(-phi);  J = rv2J(phi);  ve = Cee*(ve-J*dve); ve0=ve; pe = Cee*(pe-J*dpe);  % add AVP error
X = [phi; dve; dpe; imuerr.eb; imuerr.db];
%% update simu
len = length(imu);
avp = zeros(fix(len/nn), 10);
errX = zeros(length(avp), 10);  errR = errX;  Zk = zeros(length(avp), 13);
F = zeros(15);  Awie=askew(weie); F(1:3,1:3)=-Awie; F(4:6,4:6)=-Awie; F(7:9,4:9)=[eye(3),-Awie];
kk = timebar(nn, len);
for k=1:nn:len
    k1 = k+nn-1;  t = imu(k1,end);
    [phim, dvbm] = cnscl(imu(k:k1,:));
    % INS update in e-frame
    [peR, CenR] = blh2xyz(trj.avp(k1,7:9));  veR = CenR*trj.avp(k1,4:6)'+cross(weie,peR);
    ge = CenR*[0;0;-grav(trj.avp(k1,7:9))] + askew(weie)^2*peR;  % use real gravity
    ve = ve + rotv(-weie*nts/2,Ceb*dvbm) + (-cross(weie,ve)+ge)*nts;
    pe = pe + ((ve+ve0)/2-cross(weie,pe))*nts;  ve0 = ve;
    qeb = qupdt2(qeb, phim, weie*nts);  Ceb = q2mat(qeb);
    [pos, Cen] = xyz2blh(pe);
    avp(kk,:) = [m2att(Cen'*Ceb); Cen'*(ve-cross(weie,pe)); pos; t]';
    % INS error propagation
    F(1:3,10:12) = -Ceb;  F(4:6,1:3) = askew(ge);  F(4:6,10:15) = [-askew(ve)*Ceb,-Ceb];  F(7:9,10:12) = -askew(pe)*Ceb; % state transfer matrix
    if 1, X = expm(F*nts)*X;  else, X = (eye(15)+F*nts+F*F*nts^2/2)*X; end;
    errX(kk,:) = [X(1:9); t]';
    Cee = CenR*a2mat(trj.avp(k1,1:3))*Ceb';  phi = m2rv(Cee);  [~, JR1] = rv2J(phi);
    errR(kk,:) = [phi; JR1*(veR-Cee*ve); JR1*(peR-Cee*pe); t]';
    Zk(kk,:) = [ve-veR; cross(ve,errX(kk,1:3)')-errX(kk,4:6)'; pe-peR; cross(pe,errX(kk,1:3)')-errX(kk,7:9)'; t]';
    kk = timebar;
end
avp(kk:end,:) = [];  errX(kk:end,:) = [];  errR(kk:end,:) = [];
%% plot
myfig,
subplot(221), plot(Zk(:,end), Zk(:,1:3), '-', Zk(:,end), Zk(:,4:6), ':'); grid on
subplot(223), plot(Zk(:,end), Zk(:,7:9), '-', Zk(:,end), Zk(:,10:12), ':'); grid on
subplot(222), plot(Zk(:,end), Zk(:,1:3)-Zk(:,4:6)); grid on
subplot(224), plot(Zk(:,end), Zk(:,7:9)-Zk(:,10:12)); grid on
% avpcmpplot(trj.avp, avp, 'avp');  % SE3算法误差
% avp1 = inspure(imu, trj.avp0, 'f');
% avpcmpplot(trj.avp, avp1, 'avp');  % 传统算法误差
% avpcmpplot(avp, avp1, 'avp');  % 两者偏差
myfig,  % SE3误差模型的准确性验证
subplot(321), plot(errX(:,end), [errX(:,1:3),errR(:,1:3)]/glv.min);  xygo('phi');
subplot(322), plot(errX(:,end), [errX(:,1:3)-errR(:,1:3)]/glv.min);  xygo('phi');
subplot(323), plot(errX(:,end), [errX(:,4:6),errR(:,4:6)]);  xygo('dV');
subplot(324), plot(errX(:,end), [errX(:,4:6)-errR(:,4:6)]);  xygo('dV');
subplot(325), plot(errX(:,end), [errX(:,7:9),errR(:,7:9)]);  xygo('dP');
subplot(326), plot(errX(:,end), [errX(:,7:9)-errR(:,7:9)]);  xygo('dP');
