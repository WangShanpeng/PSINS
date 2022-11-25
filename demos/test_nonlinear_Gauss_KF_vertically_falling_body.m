% Some non-linear & Gaussian noise Kalman filters for Vertically Falling Body (VFB)
% VFB model Ref: Athans, M. 'Suboptimal state estimation for continuous-time nonlinear systems 
% from discrete noisy measurements'. IEEE Transactions on Automatic Control,1968.
% See also  vfbfx, vfbhx
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/08/2022
%% system simulator
glvs
ts = 0.01;  frq = 1/ts;
len = 30/ts;
M = 3e4; H = 3e4; gamma = 1.7e-4; r = 100;
x1 = 6e4; x2 = 6e3; x3 = 1e-3;
fpara.gamma = gamma;  fpara.ts = ts;  hpara.M = M; hpara.H = H;
X0 = [x1; x2; x3];  X = X0;
n = 3; m = 1;
xk = zeros(len,n+1);  zk = zeros(len,m);
for k=1:len
    X = vfbfx(X, fpara);
    xk(k,:) = [X', k*ts];
    zk(k,:) = vfbhx(xk(k,1), hpara) + r*randn;
end
% myfig;
% % plotyy(xk(:,end), xk(:,1), xk(:,end), xk(:,2));  legend('Alt / m', 'Vel / m/s');
% subplot(121), plot(xk(:,end), xk(:,1));  xygo('Alt / m');
% subplot(122), plot(xk(:,end), xk(:,2));  xygo('Vel / m/s');
%% EKF **********************************************************
err = xk*0;  lgstr = {};
Xk0 = [x1;x2;0]+[100;10;0].*randn(3,1);  Pk0 = diag([300, 600, 0.001])^2;
Qk = zeros(n);  Rk = r^2;
Xk = Xk0; Pk = Pk0;
for k=1:length(zk)
    [Xkk_1, Phikk_1] = vfbfx(Xk, fpara);
    Pkk_1 = Phikk_1*Pk*Phikk_1' + Qk;
    if mod(k,frq)==0
        [Zkk_1, Hk] = vfbhx(Xkk_1, hpara);
        PXZ = Pkk_1*Hk';  PZZ = Hk*PXZ+Rk;
        Kk = PXZ*PZZ^-1;
        Xk = Xkk_1 + Kk*(zk(k)-Zkk_1);
        Pk = Pkk_1 - Kk*PZZ*Kk';  Pk = (Pk+Pk')/2;
    else
        Xk = Xkk_1;  Pk = Pkk_1;
    end
	err(k,:) = [Xk'-xk(k,1:3), k*ts];
end
myfig; t = err(:,end);
subplot(3,3,1); plot(xk(:,end), xk(:,1));  xygo('Alt / m');
subplot(3,3,2); plot(xk(:,end), xk(:,2));  xygo('Vel / m/s'); title('Vertically Falling Body Simulation');
subplot(3,3,3); plot(xk(:,end), xk(:,3));  xygo('Ballistic-para / (1/m)');
subplot(3,3,[4,7]); plot(t, err(:,1), nextlinestyle(-1));  xygo('Alt est err / m');
subplot(3,3,[5,8]); plot(t, err(:,2), nextlinestyle(0));  xygo('Vel est err / m/s');
subplot(3,3,[6,9]); plot(t, err(:,3), nextlinestyle(0));  xygo('Ballistic-para est err / (1/m)');  lgstr(end+1)={'EKF'};
%% DQEKF *********************************************************
Xk = Xk0;  Pk = Pk0;
for k=1:length(zk)
    [Xkk_1, ~] = vfbfx(Xk, fpara);
    d = sqrt(diag(Pk));
    phi21 = -(exp(-gamma*(Xk(1)+d(1)/2))-exp(-gamma*(Xk(1)-d(1)/2)))*Xk(2)^2*Xk(3)*ts/d(1);
    phi22 = 1-exp(-gamma*Xk(1))*((Xk(2)+d(2)/2)^2-(Xk(2)-d(2)/2)^2)*Xk(3)*ts/d(2);
    phi23 = -exp(-gamma*Xk(1))*Xk(2)^2*ts;
    Phikk_1 = [1,-ts,0; phi21,phi22,phi23; 0,0,1];
    Pkk_1 = Phikk_1*Pk*Phikk_1' + Qk;
    if mod(k,frq)==0
        [Zkk_1, ~] = vfbhx(Xkk_1, hpara);
        d1 = sqrt(Pkk_1(1,1));
        h11 = (sqrt(M^2+(Xkk_1(1)+d1/2-H)^2)-sqrt(M^2+(Xkk_1(1)-d1/2-H)^2))/d1;
        Hk = [h11, 0, 0];
        PXZ = Pkk_1*Hk';  PZZ = Hk*PXZ+Rk;
        Kk = PXZ*PZZ^-1;
        Xk = Xkk_1 + Kk*(zk(k)-Zkk_1);
        Pk = Pkk_1 - Kk*PZZ*Kk';  Pk = (Pk+Pk')/2;
    else
        Xk = Xkk_1;  Pk = Pkk_1;
    end
	err(k,:) = [Xk'-xk(k,1:3), k*ts];
end
subplot(3,3,[4,7]); plot(t, err(:,1), nextlinestyle(1));
subplot(3,3,[5,8]); plot(t, err(:,2), nextlinestyle(0));
subplot(3,3,[6,9]); plot(t, err(:,3), nextlinestyle(0));  lgstr(end+1)={'DQEKF'};
%% EKF2 **********************************************************
err = xk*0;
Xk0 = [x1;x2;0]+[100;10;0].*randn(3,1);  Pk0 = diag([300, 600, 0.001])^2;
Qk = zeros(n);  Rk = r^2;
Xk = Xk0; Pk = Pk0;
for k=1:length(zk)
    [Xkk_1, Phikk_1, Df] = vfbfx(Xk, fpara); Pk_1=Pk;
    Pkk_1 = Phikk_1*Pk*Phikk_1' + Qk;
    for k1=1:n
        Xkk_1(k1) = Xkk_1(k1) + 1/2*trace(Df{k1}*Pk_1);
        for k2=1:n,  Pkk_1(k1,k2) = Pkk_1(k1,k2) + 1/2*trace(Df{k1}*Pk_1*Df{k2}*Pk_1);  end
    end
    if mod(k,frq)==0
        [Zkk_1, Hk, Dh] = vfbhx(Xkk_1, hpara);
        PXZ = Pkk_1*Hk';  PZZ = Hk*PXZ+Rk;
        for k1=1:m
            Zkk_1(k1) = Zkk_1(k1) + 1/2*trace(Dh{k1}*Pkk_1);
            for k2=1:m,  PZZ(k1,k2) = PZZ(k1,k2) + 1/2*trace(Dh{k1}*Pkk_1*Dh{k2}*Pkk_1);  end
        end
        Kk = PXZ*PZZ^-1;
        Xk = Xkk_1 + Kk*(zk(k)-Zkk_1);
        Pk = Pkk_1 - Kk*PZZ*Kk';  Pk = (Pk+Pk')/2;
    else
        Xk = Xkk_1;  Pk = Pkk_1;
    end
	err(k,:) = [Xk'-xk(k,1:3), k*ts];
end
subplot(3,3,[4,7]); plot(t, err(:,1), nextlinestyle(1));
subplot(3,3,[5,8]); plot(t, err(:,2), nextlinestyle(0));
subplot(3,3,[6,9]); plot(t, err(:,3), nextlinestyle(0));  lgstr(end+1)={'EKF2'};
%% GHQKF **********************************************************
Xk = Xk0; Pk = Pk0;
L = 2^n; Xikk_1 = zeros(n,L);  Zikk_1 = zeros(m,L);
pt = []; for k=0:n-1,  pt = [[pt; ones(1,2^k)], [pt; -ones(1,2^k)]]; end; wi = repmat((pi/2)^(n/2),1,L)*1/(2*pi)^(n/2);
for k=1:length(zk)
    sP = chol(Pk)';  Xik_1 = sP*pt + repmat(Xk,1,L);
    Xkk_1=zeros(n,1);  for k1=1:L, Xikk_1(:,k1) = vfbfx(Xik_1(:,k1), fpara);  Xkk_1 = Xkk_1 + wi(k1)*Xikk_1(:,k1);  end
    Pkk_1=zeros(n);  for k1=1:L, dX = Xikk_1(:,k1)-Xkk_1; Pkk_1 = Pkk_1 + wi(k1)*dX*dX'; end;  Pkk_1 = Pkk_1 + Qk;  
    if mod(k,frq)==0
        sP = chol(Pkk_1)';  Xikk_1 = sP*pt + repmat(Xkk_1,1,L);
        Zkk_1=zeros(m,1);  for k1=1:L, Zikk_1(:,k1) = vfbhx(Xikk_1(:,k1), hpara);  Zkk_1 = Zkk_1 + wi(k1)*Zikk_1(:,k1);  end
        PXZ=zeros(n,m); PZZ=zeros(m);  for k1=1:L, dX = Xikk_1(:,k1)-Xkk_1; dZ = Zikk_1(:,k1)-Zkk_1; PXZ = PXZ + wi(k1)*dX*dZ'; PZZ = PZZ + wi(k1)*dZ*dZ';  end;  PZZ = PZZ + Rk;  
        Kk = PXZ*PZZ^-1;
        Xk = Xkk_1 + Kk*(zk(k)-Zkk_1);
        Pk = Pkk_1 - Kk*PZZ*Kk';  Pk = (Pk+Pk')/2;
    else
        Xk = Xkk_1;  Pk = Pkk_1;
    end
	err(k,:) = [Xk'-xk(k,1:3), k*ts];
end
subplot(3,3,[4,7]); plot(t, err(:,1), nextlinestyle(1));
subplot(3,3,[5,8]); plot(t, err(:,2), nextlinestyle(0));
subplot(3,3,[6,9]); plot(t, err(:,3), nextlinestyle(0));   lgstr(end+1)={'GHQKF'};
%% DDF1 **********************************************************
Xk = Xk0; Pk = Pk0;
L = n;  d = sqrt(3);
for k=1:length(zk)
    Xkk_1 = vfbfx(Xk, fpara);
    sP = chol(Pk)';
    Pkk_1=zeros(n);  for k1=1:L, dX = vfbfx(Xk+sP(:,k1)*d,fpara)-vfbfx(Xk-sP(:,k1)*d,fpara); Pkk_1 = Pkk_1 + dX*dX'; end;  Pkk_1 = Pkk_1/(4*d^2) + Qk;  
    if mod(k,frq)==0
        Zkk_1 = vfbhx(Xkk_1, hpara);
        sP = chol(Pkk_1)';
        PXZ=zeros(n,m); PZZ=zeros(m);  for k1=1:L, dX = sP(:,k1); dZ = vfbhx(Xkk_1+sP(:,k1)*d,hpara)-vfbhx(Xkk_1-sP(:,k1)*d,hpara); PXZ = PXZ + dX*dZ'; PZZ = PZZ + dZ*dZ';  end;  PXZ = PXZ/(2*d); PZZ = PZZ/(4*d^2) + Rk;  
        Kk = PXZ*PZZ^-1;
        Xk = Xkk_1 + Kk*(zk(k)-Zkk_1);
        Pk = Pkk_1 - Kk*PZZ*Kk';  Pk = (Pk+Pk')/2;
    else
        Xk = Xkk_1;  Pk = Pkk_1;
    end
	err(k,:) = [Xk'-xk(k,1:3), k*ts];
end
subplot(3,3,[4,7]); plot(t, err(:,1), nextlinestyle(1));
subplot(3,3,[5,8]); plot(t, err(:,2), nextlinestyle(0));
subplot(3,3,[6,9]); plot(t, err(:,3), nextlinestyle(0));   lgstr(end+1)={'DDF1'};
%% DDF2 **********************************************************
Xk = Xk0; Pk = Pk0;
L = n;  d = sqrt(3);
for k=1:length(zk)
    sP = chol(Pk)';
    X0 = vfbfx(Xk, fpara); Xp = zeros(n); Xn = Xp; Xkk_1 = zeros(n,1);  for k1=1:L,  Xp(:,k1) = vfbfx(Xk+sP(:,k1)*d,fpara); Xn(:,k1) = vfbfx(Xk-sP(:,k1)*d,fpara); Xkk_1 = Xkk_1 + Xp(:,k1) + Xn(:,k1); end;  Xkk_1 = Xkk_1/(2*d^2) + (d^2-n)/d^2*X0;
    P1=zeros(n); P2=P1;  for k1=1:L, dX = Xp(:,k1)-Xn(:,k1); P1 = P1 + dX*dX'; dX2 = Xp(:,k1) + Xn(:,k1) - 2*X0; P2 = P2 + dX2*dX2'; end;  Pkk_1 = P1/(4*d^2) + P2*(d^2-1)/(4*d^2) + Qk;  
    if mod(k,frq)==0
        sP = chol(Pkk_1)';
        Z0 = vfbhx(Xkk_1, hpara); Zp = zeros(m,n); Zn = Zp; Zkk_1 = zeros(m,1);  for k1=1:L,  Zp(:,k1) = vfbhx(Xkk_1+sP(:,k1)*d,hpara); Zn(:,k1) = vfbhx(Xkk_1-sP(:,k1)*d,hpara); Zkk_1 = Zkk_1 + Zp(:,k1) + Zn(:,k1); end;  Zkk_1 = Zkk_1/(2*d^2) + (d^2-n)/d^2*Z0;
        PXZ=zeros(n,m); PZZ=zeros(m); P1=zeros(m); P2=P1;  for k1=1:L, dZ = Zp(:,k1)-Zn(:,k1); P1 = P1 + dZ*dZ'; dZ2 = Zp(:,k1) + Zn(:,k1) - 2*Z0; P2 = P2 + dZ2*dZ2'; PXZ = PXZ + sP(:,k1)*dZ';  end;  PZZ = P1/(4*d^2) + P2*(d^2-1)/(4*d^2) + Rk;  PXZ = PXZ/(2*d);
        Kk = PXZ*PZZ^-1;
        Xk = Xkk_1 + Kk*(zk(k)-Zkk_1);
        Pk = Pkk_1 - Kk*PZZ*Kk';  Pk = (Pk+Pk')/2;
    else
        Xk = Xkk_1;  Pk = Pkk_1;
    end
	err(k,:) = [Xk'-xk(k,1:3), k*ts];
end
subplot(3,3,[4,7]); plot(t, err(:,1), nextlinestyle(1));
subplot(3,3,[5,8]); plot(t, err(:,2), nextlinestyle(0));
subplot(3,3,[6,9]); plot(t, err(:,3), nextlinestyle(0)); lgstr(end+1)={'DDF2'};
%% CDKF **********************************************************
Xk = Xk0; Pk = Pk0;
L = 2*n;  d = 3;
for k=1:length(zk)
    Xkk_1 = vfbfx(Xk, fpara);
    sP = chol(Pk)';  sP = [sP, -sP];
    Pkk_1=zeros(n);  for k1=1:L, dX = vfbfx(Xk+sP(:,k1)*d,fpara)-Xk; Pkk_1 = Pkk_1 + dX*dX'; end;  Pkk_1 = Pkk_1/(2*d^2) + Qk;  
    if mod(k,frq)==0
        Zkk_1 = vfbhx(Xkk_1, hpara);
        sP = chol(Pkk_1)';  sP = [sP, -sP];
        PXZ=zeros(n,m); PZZ=zeros(m);  for k1=1:L, dX = sP(:,k1); dZ = vfbhx(Xkk_1+sP(:,k1)*d,hpara)-Zkk_1; PXZ = PXZ + dX*dZ'; PZZ = PZZ + dZ*dZ';  end;  PXZ = PXZ/(2*d); PZZ = PZZ/(2*d^2) + Rk;  
        Kk = PXZ*PZZ^-1;
        Xk = Xkk_1 + Kk*(zk(k)-Zkk_1);
        Pk = Pkk_1 - Kk*PZZ*Kk';  Pk = (Pk+Pk')/2;
    else
        Xk = Xkk_1;  Pk = Pkk_1;
    end
	err(k,:) = [Xk'-xk(k,1:3), k*ts];
end
subplot(3,3,[4,7]); plot(t, err(:,1), nextlinestyle(1));
subplot(3,3,[5,8]); plot(t, err(:,2), nextlinestyle(0));
subplot(3,3,[6,9]); plot(t, err(:,3), nextlinestyle(0)); lgstr(end+1)={'CDKF'};
%% UKF **********************************************************
afa = 1e-3; beta = 2; kappa = 0; lambda = afa^2*(n+kappa)-n; gamma = sqrt(n+lambda);
wm = [repmat(1/(2*gamma^2),1,2*n), lambda/gamma^2]; wc = [wm(1:end-1),wm(end)+(1-afa^2+beta)];
Xk = Xk0; Pk = Pk0;
L = 2*n+1;
for k=1:length(zk)
    sP = gamma*chol(Pk)';  Xi = [sP, -sP, zeros(n,1)]+repmat(Xk,1,L);
    Xkk_1=zeros(n,1);  for k1=1:L, Xi(:,k1) = vfbfx(Xi(:,k1),fpara); Xkk_1 = Xkk_1 + wm(k1)*Xi(:,k1); end;
    Pkk_1=zeros(n);  for k1=1:L, dX = Xi(:,k1)-Xkk_1; Pkk_1 = Pkk_1 + wc(k1)*dX*dX'; end;  Pkk_1 = Pkk_1 + Qk;  
    if mod(k,frq)==0
        sP = gamma*chol(Pkk_1)';  Xi = [sP, -sP, zeros(n,1)]+repmat(Xkk_1,1,L);
        XiZ = zeros(m,L); Zkk_1=zeros(m,1);  for k1=1:L, XiZ(:,k1) = vfbhx(Xi(:,k1),hpara); Zkk_1 = Zkk_1 + wm(k1)*XiZ(:,k1); end;
        PXZ=zeros(n,m); PZZ=zeros(m);  for k1=1:L, dX = Xi(:,k1)-Xkk_1; dZ = XiZ(:,k1)-Zkk_1; PXZ = PXZ + wc(k1)*dX*dZ'; PZZ = PZZ + wc(k1)*dZ*dZ'; end;  PZZ = PZZ + Rk;
        Kk = PXZ*PZZ^-1;
        Xk = Xkk_1 + Kk*(zk(k)-Zkk_1);
        Pk = Pkk_1 - Kk*PZZ*Kk';  Pk = (Pk+Pk')/2;
    else
        Xk = Xkk_1;  Pk = Pkk_1;
    end
	err(k,:) = [Xk'-xk(k,1:3), k*ts];
end
subplot(3,3,[4,7]); plot(t, err(:,1), nextlinestyle(1));
subplot(3,3,[5,8]); plot(t, err(:,2), nextlinestyle(0));
subplot(3,3,[6,9]); plot(t, err(:,3), nextlinestyle(0)); lgstr(end+1)={'UKF'};
%% SR-UKF ********************************************************
afa = 1e-3; beta = 2; kappa = 0; lambda = afa^2*(n+kappa)-n; gamma = sqrt(n+lambda);
wm = [repmat(1/(2*gamma^2),1,2*n), lambda/gamma^2]; wc = [wm(1:end-1),wm(end)+(1-afa^2+beta)]; swc = sqrt(abs(wc));
Xk = Xk0; sPk = chol(Pk0)'; sQk = diag(sqrt(diag(Qk))); sRk = chol(Rk)';
L = 2*n+1;
for k=1:length(zk)
    sP = gamma*sPk;  Xi = [sP, -sP, zeros(n,1)]+repmat(Xk,1,L);
    Xkk_1=zeros(n,1);  for k1=1:L, Xi(:,k1) = vfbfx(Xi(:,k1),fpara); Xkk_1 = Xkk_1 + wm(k1)*Xi(:,k1); end;
    dX=zeros(n,L);  for k1=1:L, dX(:,k1) = swc(k1)*(Xi(:,k1)-Xkk_1); end;  [q,sPkk_1] = qr([dX(:,1:L-1), sQk]');
    sPkk_1 = sPkk_1(1:n,:);
    sPkk_1 = cholupdate(sPkk_1,dX(:,end),'-')';  % dX1=dX; dX1(:,end)=-dX1(:,end); sPx = chol([dX1, sQk]*[dX1, sQk]')';
    if mod(k,frq)==0
        sP = gamma*sPkk_1;  Xi = [sP, -sP, zeros(n,1)]+repmat(Xkk_1,1,L);
        XiZ = zeros(m,L); Zkk_1=zeros(m,1);  for k1=1:L, XiZ(:,k1) = vfbhx(Xi(:,k1),hpara); Zkk_1 = Zkk_1 + wm(k1)*XiZ(:,k1); end;
        PXZ=zeros(n,m); dZ=zeros(m,L);
        for k1=1:L, dX = Xi(:,k1)-Xkk_1; dZ(:,k1) = swc(k1)*(XiZ(:,k1)-Zkk_1); PXZ = PXZ + swc(k1)*dX*dZ(:,k1)'; end;  [q,sPZZ] = qr([dZ(:,1:L-1), sRk]');
        sPZZ = sPZZ(1:m,:);
        sPZZ = cholupdate(sPZZ,dZ(:,end),'-')';  % dZ1=dZ; dZ1(:,end)=-dZ1(:,end); sPz = chol([dZ1, sRk]*[dZ1, sRk]')';
        Kk = PXZ/(sPZZ*sPZZ');
        Xk = Xkk_1 + Kk*(zk(k)-Zkk_1);
        kpzz=Kk*sPZZ'; for k1=1:m, sPkk_1 = cholupdate(sPkk_1', kpzz(:,k1), '-')'; end;  sPk = sPkk_1;
    else
        Xk = Xkk_1;  sPk = sPkk_1;
    end
	err(k,:) = [Xk'-xk(k,1:3), k*ts];
end
subplot(3,3,[4,7]); plot(t, err(:,1), nextlinestyle(1));
subplot(3,3,[5,8]); plot(t, err(:,2), nextlinestyle(0));
subplot(3,3,[6,9]); plot(t, err(:,3), nextlinestyle(0)); lgstr(end+1)={'SR-UKF'};
%% CKF3 **********************************************************
Xk = Xk0; Pk = Pk0;
L = 2*n; wi = repmat(1/(2*n),1,L);  Xikk_1 = zeros(n,L);  Zikk_1 = zeros(m,L);
for k=1:length(zk)
    sP = chol(n*Pk)';  Xik_1 = [sP,-sP] + repmat(Xk,1,L);
    Xkk_1=zeros(n,1);  for k1=1:L, Xikk_1(:,k1) = vfbfx(Xik_1(:,k1), fpara);  Xkk_1 = Xkk_1 + wi(k1)*Xikk_1(:,k1);  end
    Pkk_1=zeros(n);  for k1=1:L, dX = Xikk_1(:,k1)-Xkk_1; Pkk_1 = Pkk_1 + wi(k1)*dX*dX'; end;  Pkk_1 = Pkk_1 + Qk;  
    if mod(k,frq)==0
        sP = chol(n*Pkk_1)';  Xikk_1 = [sP,-sP] + repmat(Xkk_1,1,L);
        Zkk_1=zeros(m,1);  for k1=1:L, Zikk_1(:,k1) = vfbhx(Xikk_1(:,k1), hpara);  Zkk_1 = Zkk_1 + wi(k1)*Zikk_1(:,k1);  end
        PXZ=zeros(n,m); PZZ=zeros(m);  for k1=1:L, dX = Xikk_1(:,k1)-Xkk_1; dZ = Zikk_1(:,k1)-Zkk_1; PXZ = PXZ + wi(k1)*dX*dZ'; PZZ = PZZ + wi(k1)*dZ*dZ';  end;  PZZ = PZZ + Rk;  
        Kk = PXZ*PZZ^-1;
        Xk = Xkk_1 + Kk*(zk(k)-Zkk_1);
        Pk = Pkk_1 - Kk*PZZ*Kk';  Pk = (Pk+Pk')/2;
    else
        Xk = Xkk_1;  Pk = Pkk_1;
    end
	err(k,:) = [Xk'-xk(k,1:3), k*ts];
end
subplot(3,3,[4,7]); plot(t, err(:,1), nextlinestyle(1));
subplot(3,3,[5,8]); plot(t, err(:,2), nextlinestyle(0));
subplot(3,3,[6,9]); plot(t, err(:,3), nextlinestyle(0));  lgstr(end+1)={'CKF3'};
%% CKF5 **********************************************************
Xk = Xk0; Pk = Pk0;
L = 2*n^2+1; [pt,wi]=cubpt5(n);  Xikk_1 = zeros(n,L);  Zikk_1 = zeros(m,L);
for k=1:length(zk)
    sP = chol((n+2)*Pk)';  Xik_1 = sP*pt + repmat(Xk,1,L);
    Xkk_1=zeros(n,1);  for k1=1:L, Xikk_1(:,k1) = vfbfx(Xik_1(:,k1), fpara);  Xkk_1 = Xkk_1 + wi(k1)*Xikk_1(:,k1);  end
    Pkk_1=zeros(n);  for k1=1:L, dX = Xikk_1(:,k1)-Xkk_1; Pkk_1 = Pkk_1 + wi(k1)*dX*dX'; end;  Pkk_1 = Pkk_1 + Qk;  
    if mod(k,frq)==0
        sP = chol((n+2)*Pkk_1)';  Xikk_1 = sP*pt + repmat(Xkk_1,1,L);
        Zkk_1=zeros(m,1);  for k1=1:L, Zikk_1(:,k1) = vfbhx(Xikk_1(:,k1), hpara);  Zkk_1 = Zkk_1 + wi(k1)*Zikk_1(:,k1);  end
        PXZ=zeros(n,m); PZZ=zeros(m);  for k1=1:L, dX = Xikk_1(:,k1)-Xkk_1; dZ = Zikk_1(:,k1)-Zkk_1; PXZ = PXZ + wi(k1)*dX*dZ'; PZZ = PZZ + wi(k1)*dZ*dZ';  end;  PZZ = PZZ + Rk;  
        Kk = PXZ*PZZ^-1;
        Xk = Xkk_1 + Kk*(zk(k)-Zkk_1);
        Pk = Pkk_1 - Kk*PZZ*Kk';  Pk = (Pk+Pk')/2;
    else
        Xk = Xkk_1;  Pk = Pkk_1;
    end
	err(k,:) = [Xk'-xk(k,1:3), k*ts];
end
subplot(3,3,[4,7]); plot(t, err(:,1), nextlinestyle(1));
subplot(3,3,[5,8]); plot(t, err(:,2), nextlinestyle(0));
subplot(3,3,[6,9]); plot(t, err(:,3), nextlinestyle(0));  lgstr(end+1)={'CKF5'};
%% indrect KF ****************************************************
X = [x1;x2;1.01e-3]+[100;10;0].*randn(3,1);
Xk = zeros(n,1);  Pk = Pk0;
for k=1:length(zk)
    [X, Phikk_1] = vfbfx(X, fpara);
    Xkk_1 = Phikk_1*Xk;
    Pkk_1 = Phikk_1*Pk*Phikk_1' + Qk;
    if mod(k,frq)==0
        [Z, Hk] = vfbhx(X, hpara);
        Zkk_1 = Hk*Xkk_1;
        PXZ = Pkk_1*Hk';  PZZ = Hk*PXZ+Rk;
        Kk = PXZ*PZZ^-1;
        Xk = Xkk_1 + Kk*(zk(k)-Z-Zkk_1);
        Pk = Pkk_1 - Kk*PZZ*Kk';  Pk = (Pk+Pk')/2;
        X = X+Xk;  Xk = zeros(n,1);   % feedback
    else
        Xk = Xkk_1;  Pk = Pkk_1;
    end
	err(k,:) = [X'-xk(k,1:3), k*ts];
end
subplot(3,3,[4,7]); plot(t, err(:,1), nextlinestyle(1));
subplot(3,3,[5,8]); plot(t, err(:,2), nextlinestyle(0));
subplot(3,3,[6,9]); plot(t, err(:,3), nextlinestyle(0)); lgstr(end+1)={'indirectKF'};
legend(lgstr);

