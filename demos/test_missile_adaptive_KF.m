% 四种自适应滤波仿真（针对常速度模型的导弹状态估计，问题描述参见'捷联惯导算法与组合导航原理'书练习题35）.
%	AKF   - 常规自适应卡尔曼滤波
%	MCKF  - MCKF最大相关熵卡尔曼滤波
%	RSTKF - RSTKF鲁棒学生t卡尔曼滤波
%	SSMKF - SSMKF统计相似度量卡尔曼滤波
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/03/2022
%% 参数设置
Ft = [0 -1; 0 0];  Gt = [0; 1];
q = 0.05;
Ts = 0.5;
Phi = eye(2)+Ft*Ts;  Gamma = Gt;  Qk = q*Ts;
Hk = [1 0];  Rk = 50^2;
%% 轨迹仿真
s0 = 100000; v0 = 300;
Xk = [s0; v0];
Xkk = zeros(fix(s0/300/Ts),2); Zkk = Xkk(:,1);
k = 1;
while Xk(1)>0
    Xk = Phi*Xk + Gamma*randn(1)*sqrt(Qk);
    Zk = Hk*Xk + htwn(0.1,100)*sqrt(Rk);  % 厚尾量测噪声
    Xkk(k,:) = Xk';
    Zkk(k,:) = Zk; k = k+1;
end
Xkk(k-1:end,:) = [];  Zkk(k-1:end,:) = [];
figure
subplot(211), plot([Xkk(:,1),Zkk]); grid on; xlabel('k'); ylabel('距离/m'); legend('真实距离sk', '观测距离Zk');
subplot(212), plot(Xkk(:,2)); grid on; xlabel('k'); ylabel('速度/m/s'); legend('真实速度vk');
%% 四种滤波 KF/MCKF/RSTKF/SSMKF
akf.xk = [s0+100; v0+10];  akf.Pxk = diag([100, 10])^2;
akf.Phikk_1 = Phi;   akf.Gammak = Gamma;   akf.Qk = Qk;
akf.Rk = Rk;  akf.Hk = Hk;
ares = zeros(length(Zkk),5);
mkf = akf; rkf = akf; skf = akf;   mres = ares; rres = ares; sres = ares; 
for k=1:length(Zkk)
    akf = akfupdate(akf, Zkk(k), 'B', 'AKF');       ares(k,:) = [akf.xk; diag(akf.Pxk); akf.lambda];
    mkf = akfupdate(mkf, Zkk(k), 'B', 'MCKF');      mres(k,:) = [mkf.xk; diag(mkf.Pxk); mkf.lambda];
    rkf = akfupdate(rkf, Zkk(k), 'B', 'RSTKF');     rres(k,:) = [rkf.xk; diag(rkf.Pxk); rkf.lambda];
    skf = akfupdate(skf, Zkk(k), 'B', 'SSMKF');     sres(k,:) = [skf.xk; diag(skf.Pxk); skf.lambda];
end
figure
subplot(221), plot(Zkk-Xkk(:,1)); grid on; xlabel('k'); ylabel('距离/m'); legend('厚尾距离观测误差Zk-sk');
subplot(223), plot([Xkk(:,2),ares(:,2),mres(:,2),rres(:,2),sres(:,2)]); grid on; xlabel('k'); ylabel('速度/m/s'); 
    legend('真实速度vk', 'AKF估计','MCKF估计','RSTKF估计','SSMKF估计');
    perr = [ares(:,1)-Xkk(:,1),mres(:,1)-Xkk(:,1),rres(:,1)-Xkk(:,1),sres(:,1)-Xkk(:,1)];
subplot(222), plot(perr); hold on, grid on; xlabel('k'); plot(sqrt([ares(:,3),mres(:,3),rres(:,3),sres(:,3)])); ylabel('距离误差/m');
    perr=mean(abs(perr(100:end,:))); legend(sprintf('AKF估计%.3f',perr(1)),sprintf('MCKF估计%.3f',perr(2)),sprintf('RSTKF估计%.3f',perr(3)),sprintf('SSMKF估计%.3f',perr(4)));
    verr = [ares(:,2)-Xkk(:,2),mres(:,2)-Xkk(:,2),rres(:,2)-Xkk(:,2),sres(:,2)-Xkk(:,2)];
subplot(224), plot(verr); hold on, grid on; xlabel('k'); plot(sqrt([ares(:,4),mres(:,4),rres(:,4),sres(:,4)])); ylabel('速度误差/m/s');
    verr = mean(abs(verr(100:end,:))); legend(sprintf('AKF估计%.3f',verr(1)),sprintf('MCKF估计%.3f',verr(2)),sprintf('RSTKF估计%.3f',verr(3)),sprintf('SSMKF估计%.3f',verr(4)));
% figure, plot([ares(:,5),mres(:,5),rres(:,5),sres(:,5)]), grid on; xlabel('k'); ylabel('迭代次数'); legend('AKF估计','MCKF估计','RSTKF估计','SSMKF估计');


