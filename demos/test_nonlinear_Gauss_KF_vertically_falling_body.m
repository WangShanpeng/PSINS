% Some non-linear & Gaussian noise Kalman filters for Vertically Falling Body (VFB)
% VFB model Ref: Athans, M. 'Suboptimal state estimation for continuous-time nonlinear systems 
% from discrete noisy measurements'. IEEE Transactions on Automatic Control,1968.
% See also  vfbfx, vfbhx
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/08/2022, 01/01/2023
%% VFB system simulator ******************************************
glvs;
Ts = 0.01;  len = fix(30/Ts);
gamma = 1.7e-4; g = 9.8; M = 3e4; H = 3e4; rk = 100;
tpara.gamma = gamma; tpara.g = g; tpara.Ts = Ts; tpara.M = M; tpara.H = H;
x10 = 6e4; x20 = 6e3; x30 = 1e-3;  Xn = [x10; x20; x30];
n = 3; m = 1;
xk = zeros(len,n);  zk = zeros(len,m);  t = (1:len)'*Ts;
for k=1:len
    Xn = vfbfx(Xn, tpara);
    xk(k,:) = Xn';
    zk(k,:) = vfbhx(xk(k,1), tpara, rk);
end
% myfig;
% subplot(331); plot(t, [xk(:,1),zk], 'linewidth',2);  xygo('X_{1,k}, Z_k / m');  legend('X_{1,k}','Z_k');
% subplot(332); plot(t, xk(:,2), 'linewidth',2);  xygo('X_{2,k} / (m.s^{-1})'); title('Vertically Falling Body Simulation');  ylim([0,8000]);
% subplot(333); plot(t, exp(-gamma*xk(:,1)).*xk(:,2).^2.*xk(:,3)/g, 'linewidth',2);  xygo('f_{r,k} / g');
%% EKF ***********************************************************
T = 1;  zint = fix(T/Ts);  % measurement interval/frequency
res = zeros(len,2*n);
Qk = zeros(n);  Rk = rk^2;
Xk0 = [x10;x20;0]+[1000;600;0.001].*randn(3,1);
Pk0 = diag([1000,600,0.001])^2; % init KF X0,P0
initSmall=1;
if initSmall==1,  Xk0(3) = 0.001+0.0001*randn(1);  Pk0(3,3) = 0.0001^2;  end  % ***
Xk = Xk0;  Pk = Pk0;
for k=1:len
    [Xkk_1, Phikk_1] = vfbfx(Xk, tpara);  % time update
    Pkk_1 = Phikk_1*Pk*Phikk_1' + Qk;
    if mod(k,zint)==0  % meas update
        [Zkk_1, Hk] = vfbhx(Xkk_1, tpara);
        PXZ = Pkk_1*Hk';  PZZ = Hk*PXZ+Rk;  Kk = PXZ*PZZ^-1;
        Xk = Xkk_1 + Kk*(zk(k)-Zkk_1);
        Pk = Pkk_1 - Kk*PZZ*Kk';  Pk = (Pk+Pk')/2;
    else
        Xk = Xkk_1;  Pk = Pkk_1;
    end
	res(k,:) = [Xk; diag(Pk)]';
end
ress = {}; lgstr = {};
ress{end+1}=res; lgstr(end+1)={'EKF'};
%% DQEKF *********************************************************
Xk = Xk0;  Pk = Pk0;
for k=1:len
    [Xkk_1, Phi1] = vfbfx(Xk, tpara);
    d = sqrt(diag(Pk));
    phi21 = -(exp(-gamma*(Xk(1)+d(1)/2))-exp(-gamma*(Xk(1)-d(1)/2)))* ...
            Xk(2)^2*Xk(3)*Ts/d(1);
    phi22 = 1-exp(-gamma*Xk(1))*((Xk(2)+d(2)/2)^2-(Xk(2)-d(2)/2)^2)* ...
            Xk(3)*Ts/d(2);
    phi23 = -exp(-gamma*Xk(1))*Xk(2)^2*Ts;
    Phikk_1 = [1,-Ts,0; phi21,phi22,phi23; 0,0,1];
    Pkk_1 = Phikk_1*Pk*Phikk_1' + Qk;
    if mod(k,zint)==0
        [Zkk_1, H1] = vfbhx(Xkk_1, tpara);
        d = sqrt(Pkk_1(1,1));
        h11 = (sqrt(M^2+(Xkk_1(1)+d/2-H)^2)-sqrt(M^2+(Xkk_1(1)-d/2-H)^2))/d;
        Hk = [h11, 0, 0];
        PXZ = Pkk_1*Hk';  PZZ = Hk*PXZ+Rk;  Kk = PXZ*PZZ^-1;
        Xk = Xkk_1 + Kk*(zk(k)-Zkk_1);
        Pk = Pkk_1 - Kk*PZZ*Kk';  Pk = (Pk+Pk')/2;
    else
        Xk = Xkk_1;  Pk = Pkk_1;
    end
	res(k,:) = [Xk; diag(Pk)]';
end
ress{end+1}=res; lgstr(end+1)={'DQEKF'};
%% EKF2 **********************************************************
Xk = Xk0; Pk = Pk0;
for k=1:len
    [Xkk_1, Phikk_1, Df] = vfbfx(Xk, tpara); Pk_1 = Pk;
    Pkk_1 = Phikk_1*Pk*Phikk_1' + Qk;
    for k1=1:n
        Xkk_1(k1) = Xkk_1(k1) + 1/2*trace(Df{k1}*Pk_1);
        for k2=1:n,
            Pkk_1(k1,k2) = Pkk_1(k1,k2) + 1/2*trace(Df{k1}*Pk_1*Df{k2}*Pk_1);  end
    end
    if mod(k,zint)==0
        [Zkk_1, Hk, Dh] = vfbhx(Xkk_1, tpara);
        PXZ = Pkk_1*Hk';  PZZ = Hk*PXZ+Rk;
        for k1=1:m
            Zkk_1(k1) = Zkk_1(k1) + 1/2*trace(Dh{k1}*Pkk_1);
            for k2=1:m,
                PZZ(k1,k2) = PZZ(k1,k2) + 1/2*trace(Dh{k1}*Pkk_1*Dh{k2}*Pkk_1);  end
        end
        Kk = PXZ*PZZ^-1;
        Xk = Xkk_1 + Kk*(zk(k)-Zkk_1);
        Pk = Pkk_1 - Kk*PZZ*Kk';  Pk = (Pk+Pk')/2;
    else
        Xk = Xkk_1;  Pk = Pkk_1;
    end
	res(k,:) = [Xk; diag(Pk)]';
end
ress{end+1}=res; lgstr(end+1)={'EKF2'};
%% Iterative EKF *************************************************
Xk = Xk0; Pk = Pk0;
for k=1:len
    [Xkk_1, Phikk_1] = vfbfx(Xk, tpara);
    Pkk_1 = Phikk_1*Pk*Phikk_1' + Qk;
    if mod(k,zint)==0
        [Zkk_1, Hk] = vfbhx(Xkk_1, tpara);
        PXZ = Pkk_1*Hk';  PZZ = Hk*PXZ+Rk;  Kk = PXZ*PZZ^-1;
        Xk1 = Xkk_1 + Kk*(zk(k)-Zkk_1);  % pre filtering
        Xk_1k = Xk + Pk*Phikk_1'*invbc(Pkk_1)*(Xk1-Xkk_1);  % RTS
        [Xkk_1, Phikk_1] = vfbfx(Xk_1k, tpara);  Xkk_1 = Xkk_1+Phikk_1*(Xk-Xk_1k);
        Pkk_1 = Phikk_1*Pk*Phikk_1' + Qk;
        [Zkk_1, Hk] = vfbhx(Xk1, tpara);  Zkk_1 = Zkk_1+Hk*(Xkk_1-Xk1);
        PXZ = Pkk_1*Hk';  PZZ = Hk*PXZ+Rk;  Kk = PXZ*PZZ^-1;
        Xk = Xkk_1 + Kk*(zk(k)-Zkk_1);
        Pk = Pkk_1 - Kk*PZZ*Kk';  Pk = (Pk+Pk')/2;
    else
        Xk = Xkk_1;  Pk = Pkk_1;
    end
	res(k,:) = [Xk; diag(Pk)]';
end
ress{end+1}=res; lgstr(end+1)={'IEKF'};
%% CDKF1 *********************************************************
Xk = Xk0; Pk = Pk0;
L = n;  d = sqrt(3);
for k=1:len
    Xkk_1 = vfbfx(Xk, tpara);
    sP = chol(Pk,'lower');   Pkk_1 = Qk;
    for k1=1:L, dX = vfbfx(Xk+sP(:,k1)*d,tpara)-vfbfx(Xk-sP(:,k1)*d,tpara);
                Pkk_1 = Pkk_1 + dX*dX'/(4*d^2); end; 
    if mod(k,zint)==0
        Zkk_1 = vfbhx(Xkk_1, tpara);
        sP = chol(Pkk_1,'lower');  PXZ = zeros(n,m); PZZ = Rk;
        for k1=1:L, dX = sP(:,k1);
            dZ = vfbhx(Xkk_1+sP(:,k1)*d,tpara)-vfbhx(Xkk_1-sP(:,k1)*d,tpara);
            PXZ = PXZ + dX*dZ'/(2*d); PZZ = PZZ + dZ*dZ'/(4*d^2);   end;
        Kk = PXZ*PZZ^-1;
        Xk = Xkk_1 + Kk*(zk(k)-Zkk_1);
        Pk = Pkk_1 - Kk*PZZ*Kk';  Pk = (Pk+Pk')/2;
    else
        Xk = Xkk_1;  Pk = Pkk_1;
    end
	res(k,:) = [Xk; diag(Pk)]';
end
ress{end+1}=res; lgstr(end+1)={'CDKF1'};
%% CDKF2 *********************************************************
Xk = Xk0; Pk = Pk0;
L = n;  d = sqrt(3);
for k=1:len
    sP = chol(Pk,'lower');
    X0 = vfbfx(Xk, tpara); Xp = zeros(n); Xn = Xp; Xkk_1 = zeros(n,1);
    for k1=1:L, Xp(:,k1) = vfbfx(Xk+sP(:,k1)*d,tpara);
                Xn(:,k1) = vfbfx(Xk-sP(:,k1)*d,tpara);
                Xkk_1 = Xkk_1+Xp(:,k1)+Xn(:,k1); end;
    Xkk_1 = (d^2-n)/d^2*X0 + Xkk_1/(2*d^2);
    P1 = zeros(n); P2 = P1;
    for k1=1:L, dX = Xp(:,k1)-Xn(:,k1); P1 = P1+dX*dX';
                dX2 = Xp(:,k1)+Xn(:,k1)-2*X0; P2 = P2+dX2*dX2'; end;
    Pkk_1 = P1/(4*d^2) + P2*(d^2-1)/(4*d^2) + Qk;  
    if mod(k,zint)==0
        sP = chol(Pkk_1,'lower');
        Z0 = vfbhx(Xkk_1, tpara); Zp = zeros(m,n); Zn = Zp; Zkk_1 = zeros(m,1);
        for k1=1:L, Zp(:,k1) = vfbhx(Xkk_1+sP(:,k1)*d,tpara);
                    Zn(:,k1) = vfbhx(Xkk_1-sP(:,k1)*d,tpara);
                    Zkk_1 = Zkk_1+Zp(:,k1)+Zn(:,k1); end;
        Zkk_1 = (d^2-n)/d^2*Z0 + Zkk_1/(2*d^2);
        PXZ = zeros(n,m); P1 = zeros(m); P2 = P1;
        for k1=1:L, dZ = Zp(:,k1)-Zn(:,k1); P1 = P1+dZ*dZ';
                    dZ2 = Zp(:,k1)+Zn(:,k1)-2*Z0; P2 = P2+dZ2*dZ2';
                    PXZ = PXZ+sP(:,k1)*dZ'; end;
        PZZ = P1/(4*d^2) + P2*(d^2-1)/(4*d^2) + Rk;  PXZ = PXZ/(2*d);
        Kk = PXZ*PZZ^-1;
        Xk = Xkk_1 + Kk*(zk(k)-Zkk_1);
        Pk = Pkk_1 - Kk*PZZ*Kk';  Pk = (Pk+Pk')/2;
    else
        Xk = Xkk_1;  Pk = Pkk_1;
    end
	res(k,:) = [Xk; diag(Pk)]';
end
ress{end+1}=res; lgstr(end+1)={'CDKF2'};
%% FBDKF *********************************************************
Xk = Xk0; Pk = Pk0;
L = 2*n;  d = sqrt(3);
for k=1:len
    Xkk_1 = vfbfx(Xk, tpara);
    sP = chol(Pk,'lower');  sP = [sP, -sP];
    Pkk_1=zeros(n);
    for k1=1:L, dX = vfbfx(Xk+sP(:,k1)*d,tpara)-Xkk_1; Pkk_1 = Pkk_1+dX*dX'; end;
    Pkk_1 = Pkk_1/(2*d^2) + Qk;  
    if mod(k,zint)==0
        Zkk_1 = vfbhx(Xkk_1, tpara);
        sP = chol(Pkk_1,'lower');  sP = [sP, -sP];
        PXZ=zeros(n,m); PZZ=zeros(m);
        for k1=1:L, dX = sP(:,k1); dZ = vfbhx(Xkk_1+sP(:,k1)*d,tpara)-Zkk_1;
                    PXZ = PXZ+dX*dZ'; PZZ = PZZ+dZ*dZ';  end;
        PXZ = PXZ/(2*d); PZZ = PZZ/(2*d^2) + Rk;
        Kk = PXZ*PZZ^-1;
        Xk = Xkk_1 + Kk*(zk(k)-Zkk_1);
        Pk = Pkk_1 - Kk*PZZ*Kk';  Pk = (Pk+Pk')/2;
    else
        Xk = Xkk_1;  Pk = Pkk_1;
    end
	res(k,:) = [Xk; diag(Pk)]';
end
ress{end+1}=res; lgstr(end+1)={'FBDKF'};
%% GHQKF3 ********************************************************
[U, wm] = ghpoint(n, 3);  wc = wm;  % Gaussian point matrix & weights
GCU_KF_Frame;
ress{end+1}=res; lgstr(end+1)={'GHQKF3'};
%% GHQKF5 ********************************************************
[U, wm] = ghpoint(n, 5);  wc = wm;
GCU_KF_Frame;
ress{end+1}=res; lgstr(end+1)={'GHQKF5'};
%% CKF3 **********************************************************
[U, wm] = cubpoint(n, 3);  wc = wm;
GCU_KF_Frame;
ress{end+1}=res; lgstr(end+1)={'CKF3'};
%% CKF5 **********************************************************
[U, wm] = cubpoint(n, 5);  wc = wm;
GCU_KF_Frame;
ress{end+1}=res; lgstr(end+1)={'CKF5'};
%% UKF ***********************************************************
[U, wm, wc] = utpoint(n);
GCU_KF_Frame;
ress{end+1}=res; lgstr(end+1)={'UKF'};
%% SR-UKF ********************************************************
[U, wm, wc, gmm] = utpoint(n);  swc = sqrt(abs(wc));
L = length(wm); Xk = Xk0; sPk = chol(Pk0,'lower'); sQk = zeros(3); sRk = chol(Rk,'lower');
for k=1:len
    sP = gmm*sPk;  Xi = [sP, -sP, zeros(n,1)]+repmat(Xk,1,L);  % sigma points
%     Xi = sPk*U + repmat(Xk,1,L);
    Xkk_1=zeros(n,1);
    for k1=1:L, Xi(:,k1) = vfbfx(Xi(:,k1),tpara);
                Xkk_1 = Xkk_1+wm(k1)*Xi(:,k1); end;  % mean
    dX=zeros(n,L); for k1=1:L, dX(:,k1) = swc(k1)*(Xi(:,k1)-Xkk_1); end;
    [q,sPkk_1] = qr([dX(:,1:L-1), sQk]',0);
    sPkk_1 = mycholupdate(sPkk_1,dX(:,end),-1)';  % var
    if mod(k,zint)==0
        sP = gmm*sPkk_1;  dX = [sP, -sP, zeros(n,1)];
        Xi = dX+repmat(Xkk_1,1,L);  XiZ = zeros(m,L);
%         dX = sPkk_1*U;  Xi = dX + repmat(Xkk_1,1,L);  XiZ = zeros(m,L);
        Zkk_1=zeros(m,1);
        for k1=1:L, XiZ(:,k1) = vfbhx(Xi(:,k1),tpara);
                    Zkk_1 = Zkk_1+wm(k1)*XiZ(:,k1); end;
        PXZ=zeros(n,m); dZ=zeros(m,L);
        for k1=1:L, dZ(:,k1) = swc(k1)*(XiZ(:,k1)-Zkk_1);
                    PXZ = PXZ+swc(k1)*dX(:,k1)*dZ(:,k1)'; end;
        [q,sPZZ] = qr([dZ(:,1:L-1), sRk]',0);
        sPZZ = mycholupdate(sPZZ,dZ(:,end),-1)';
        Kk = PXZ/(sPZZ*sPZZ');
        Xk = Xkk_1 + Kk*(zk(k)-Zkk_1);
        sPk = mycholupdate(sPkk_1', Kk*sPZZ, -1)';
    else
        Xk = Xkk_1;  sPk = sPkk_1;
    end
	res(k,:) = [Xk; diag(sPk*sPk')]';
end
ress{end+1}=res; lgstr(end+1)={'SRUKF'};
%% InDrect KF ****************************************************
Xn = Xk0;
Xk = zeros(n,1);  Pk = Pk0;
for k=1:len
    [Xn, Phikk_1] = vfbfx(Xn, tpara);
    Xkk_1 = Phikk_1*Xk;
    Pkk_1 = Phikk_1*Pk*Phikk_1' + Qk;
    if mod(k,zint)==0
        [Z, Hk] = vfbhx(Xn, tpara);
        Zkk_1 = Hk*Xkk_1;
        PXZ = Pkk_1*Hk';  PZZ = Hk*PXZ+Rk;  Kk = PXZ*PZZ^-1;
        Xk = Xkk_1 + Kk*(zk(k)-Z-Zkk_1);
        Pk = Pkk_1 - Kk*PZZ*Kk';  Pk = (Pk+Pk')/2;
        Xn = Xn+Xk;  Xk = zeros(n,1);   % feedback
    else
        Xk = Xkk_1;  Pk = Pkk_1;
    end
	res(k,:) = [Xn; diag(Pk)]';
end
ress{end+1}=res; lgstr(end+1)={'IDKF'};
%% plot **********************************************************
myfig; p=1:10:len; tp = t(p);
subplot(331); plot(tp, [xk(p,1),zk(p,1)]);  xygo('X_{1,k}, Z_k / m');  legend('X_{1,k}','Z_k');
subplot(332); plot(tp, xk(p,2));  xygo('X_{2,k} / ( m.s^{-1} )'); ylim([0,8000]); title('Vertically Falling Body Simulation');
% subplot(333); plot(t, xk(p,3));  xygo('X_{3,k} / m^{-1}');  ylim([0,2*xk(1,3)]);
subplot(333); plot(tp, exp(-gamma*xk(p,1)).*xk(p,2).^2.*xk(p,3)/g);  xygo('f_{r,k} / g');
nextlinestyle(-1); xx=[]; kk = 12:13; 1:14; 8:11; [5,7,10,13]; [10,8,3,12,6,2,5,7,1]; 
for k=kk
    subplot(334); plot(tp, ress{k}(p,1)-xk(p,1), nextlinestyle(1)); xygo('Err_{X1,k} / m');
    subplot(335); plot(tp, ress{k}(p,2)-xk(p,2), nextlinestyle(0)); xygo('Err_{X2,k} / ( m.s^{-1} )');
    subplot(336); plot(tp, ress{k}(p,3)-xk(p,3), nextlinestyle(0)); xygo('Err_{X3,k} / m^{-1}');
    subplot(337); plot(tp, sqrt(ress{k}(p,4)), nextlinestyle(0)); xygo('\surdP_{X1,k} / m');
    subplot(338); plot(tp, sqrt(ress{k}(p,5)), nextlinestyle(0)); xygo('\surdP_{X2,k} / ( m.s^{-1} )');
    subplot(339); plot(tp, sqrt(ress{k}(p,6)), nextlinestyle(0)); xygo('\surdP_{X3,k} / m^{-1}');
    xx(k,:)=ress{k}(end-1,1:3)-xk(end-1,1:3);
end
subplot(336); legend(lgstr{kk});
% CKF5「GHQKF5 > CKF3「GHQKF3「UKF=SRUKF「EKF2「CDKF2 > DQEKF「CDKF1「FBDKF「EKF=IDKF「IEKF  
myfig; kk=1;
for k=[11,10,8,3,12,6,2,5,7,1,4], subplot(4,3,kk); plot(t, ress{k}(:,1:3)-ress{9}(:,1:3)); legend([lgstr{k},' Err']); kk=kk+1; end
% [~,idx]=sort(xx(:,1)); xx=[xx(idx,:),idx];
%%
return;
myfig;
subplot(221), plot(ress{1}(:,1:3)-ress{14}(:,1:3)); legend('EKF-IDKF');
subplot(222), plot(ress{12}(:,1:3)-ress{13}(:,1:3)); legend('UKF-SRUKF');
subplot(223), plot(ress{9}(:,1:3)-ress{11}(:,1:3)); legend('GHQKF5-CKF5');
subplot(224), plot(ress{5}(:,1:3)-ress{7}(:,1:3)); legend('EKF-DQEKF');

rmsdata = [];  meandata = []; xx1=[]; xx2=[]; xx3=[];
for j=1:10,
    test_nonlinear_Gauss_KF_vertically_falling_body; xx(:,1)',
    [rmsdata, meandata, stddata] = rmsupdate(rmsdata, meandata, appendt(xx(:,1:3)), j);
    xx1=[xx1,xx(:,1)];  xx2=[xx2,xx(:,2)];  xx3=[xx3,xx(:,3)];
end

myfig, subplot(131), plot(xx1'), hold on, plot(xx1(end,:),'linewidth',3);
subplot(132), plot(xx2'), hold on, plot(xx2(end,:),'linewidth',3);
subplot(133), plot(xx3'), hold on, plot(xx3(end,:),'linewidth',3);

