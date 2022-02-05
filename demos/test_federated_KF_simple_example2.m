% A simple example test for federalted Kalman filter (FKF).
% State space models:
%  |X1k|   | 1 0 | |X1k_1|   |Q1k|      |Zk1|   | 1 1 | |X1k|   |V1k|
%  |X2k| = | 0 1 | |X2k_1| + |Q2k|  ,   |Z2k| = | 1 0 | |X2k| + |V2k| 
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/01/2022
glvs;
% setting
X0 = [1; 2];
P0 = diag([10 10])^2;
len = 300;  nmeas = 1;
%% centralized KF init
ckf = [];
ckf.Phikk_1 = [1 0; 0 1];  ckf.Qt = 0*diag([0.1 0.1])^2;
ckf.Hk = [1 1; 1 0];  [m,n] = size(ckf.Hk);  ckf.Rk = diag(0.1*ones(m,1))^2;
ckf.Pxk = P0;
ckf = kfinit0(ckf, 1);
%% federated KF init
fkf = fkfinit(ckf, {[1,2]; [1]},  {[1];[2]}, [1/3; 1/2], 3);
%% 
[xtrue, cxk,cpk, xk1,pk1, xk2,pk2, fxk,fpk] = prealloc(len,  ckf.n+1, ckf.n,ckf.n, fkf{1}.n,fkf{1}.n, fkf{2}.n,fkf{2}.n, fkf{3}.n,fkf{3}.n);
ki = timebar(1, len, '2-state centralized v.s. federated KF simulation.');
Xk = X0;
xtrue(ki,:) = [Xk; ki]';
cxk(ki,:)  = ckf.xk';     cpk(ki,:)  = diag(ckf.Pxk)';
xk1(ki,:)  = fkf{1}.xk';  pk1(ki,:)  = diag(fkf{1}.Pxk)';
xk2(ki,:)  = fkf{2}.xk';  pk2(ki,:)  = diag(fkf{2}.Pxk)';
fxk(ki,:)  = fkf{3}.xk';  fpk(ki,:)  = diag(fkf{3}.Pxk)';
for k=1:1:len
    %% state & meas simulation
%     if k<100, h21=0; elseif k<200, h21=1; else, h21=2; end
%     ckf.Hk(2,2) = h21;  % ckf.Hk(1,2) = 0; ckf.Hk(2,3) = 0;
    Xk = ckf.Phikk_1 * Xk  + sqrt(diag(ckf.Qk)).*randn(n,1);
    Zk = ckf.Hk * Xk + sqrt(diag(ckf.Rk)).*randn(m,1);
    %% centralized & federated KF
    if mod(k,nmeas)==0
        ckf = kfupdate(ckf, Zk); 
        [fkf, testHk] = fkfupdate(ckf, fkf, Zk);
    else
        ckf = kfupdate(ckf);
        fkf = fkfupdate(ckf, fkf);
    end
   %% save results
    ki = timebar;
    xtrue(ki,:) = [Xk; ki]';
    cxk(ki,:)  = ckf.xk'+0.01;     cpk(ki,:)  = diag(ckf.Pxk)';
    xk1(ki,:)  = fkf{1}.xk';  pk1(ki,:)  = diag(fkf{1}.Pxk)';
    xk2(ki,:)  = fkf{2}.xk';  pk2(ki,:)  = diag(fkf{2}.Pxk)';
    fxk(ki,:)  = fkf{3}.xk';  fpk(ki,:)  = diag(fkf{3}.Pxk)';
end
%% plot
x1 = cxk(:,1);  x2 = cxk(:,2);  sx1 = {'CKF'}; sx2 = sx1;
p1 = cpk(:,1);  p2 = cpk(:,2);  sp1 = {'CKF'}; sp2 = sp1;
xk = {xk1,xk2,fxk};  pk = {pk1, pk2, fpk};  strkfi = {'KF1', 'KF2', 'FKF'};
for k=1:3
    [~,i1] = intersect(fkf{k}.subn,1); [~,i2] = intersect(fkf{k}.subn,2); [~,i3] = intersect(fkf{k}.subn,3);
    if ~isempty(i1), x1=[x1, xk{k}(:,i1)];  p1=[p1,pk{k}(:,i1)]; sx1{end+1}=strkfi{k}; sp1{end+1}=strkfi{k}; end
    if ~isempty(i2), x2=[x2, xk{k}(:,i2)];  p2=[p2,pk{k}(:,i2)]; sx2{end+1}=strkfi{k}; sp2{end+1}=strkfi{k}; end
end
x1 = [x1, xtrue(:,1)];  x2 = [x2, xtrue(:,2)];  sx1{end+1} = 'Xture';  sx2{end+1} = 'Xture';
t = xtrue(:,end)-1;
myfig, 
subplot(221), plot(t, x1); legend(sx1); xygo('X_1')
subplot(222), semilogy(t, sqrt(p1)); legend(sp1); xygo('sqrt(PX_1)');
subplot(223), plot(t, x2); legend(sx2); xygo('X_2')
subplot(224), semilogy(t, sqrt(p2)); legend(sp2); xygo('sqrt(PX_2)');

