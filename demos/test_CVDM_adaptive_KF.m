% KF+五种自适应滤波仿真（针对常速度动态模型(constant velociyt dynamic model, CVDM)的导弹状态估计
% 问题描述参见'捷联惯导算法与组合导航原理'书练习题35）.
%	RkKnownKF - 噪声准确已知的卡尔曼滤波（作为最优参考）
%	MSHAKF - 改进Sage-Husa自适应卡尔曼滤波
%	MCKF  - 最大相关熵卡尔曼滤波, 几种变分贝叶斯估计方法
%	RSTKF - 鲁棒学生t卡尔曼滤波
%	SSMKF - 统计相似度量卡尔曼滤波
%   CERKF - 计算高效鲁棒卡尔曼滤波
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/04/2022
%% 参数设置
Ft = [0 1; 0 0];  Gt = [0; 1];
q = 0.05;
Ts = .2;
Phi = eye(2)+Ft*Ts;  Gamma = Gt;  Qk = q*Ts;
Hk = [1 0];  Rk = 10^2;                          % distance measurement
% Hk = [1 0; 0 1];  Rk = diag([5^2; 1^2]);       % distance+velocity measurement
[m, n] = size(Hk);
s0 = 10; v0 = 2;
len = 500;
sp = zeros(len,6); sv = sp;
for MCn=1:10  % Monte Carlo runs
    %% 轨迹仿真
    Xk = [s0; v0];
	Xkk = zeros(len,n); Zkk = zeros(len,m); Rkk = Zkk; wnk = Rkk;
    for k=1:len
        Xk = Phi*Xk + Gamma*sqrt(Qk)*randn(1);
        Xkk(k,:) = Xk';
        p=0.1; s=10;
        [wn,s] = htwn(p,s,m);  Rkk(k,:) = (s.^2)'.*diag(Rk)';  % 厚尾量测噪声
        Zk = Hk*Xkk(k,:)' + sqrt(Rk)*wn;  wnk(k,:) = sqrt(Rk)*wn;
        Zkk(k,:) = Zk';
    end
    %% 六种滤波 KF/MSHAKF/MCKF/RSTKF/SSMKF
    kf.xk = [s0+10*randn(1); v0+1*randn(1)]*0;  kf.Pxk = diag([10, 1])^2;
    kf.Phikk_1 = Phi;   kf.Gammak = Gamma;   kf.Qk = Qk;
    kf.Rk = Rk*1;  kf.Hk = Hk;  [kf.m, kf.n] = size(Hk);  kf.measIter = 2; kf.betak = ones(kf.m,1); kf.Rmin = diag(Rk); kf.Rmax = diag(Rk)*100;
    res = zeros(length(Zkk),5);
    akf = kf; mkf = kf; rkf = kf; skf = kf; ckf = kf;   ares = res; mres = res; rres = res; sres = res; cres = res;
	bs = repmat([0.0,3],kf.m,1);
    for k=1:length(Zkk)
        kf.Rk = diag(Rkk(k,:));  % myfig(sqrt(Rkk));
        kf  = akfupdate(kf,  Zkk(k,:)', 'B', 'KF');             res(k,:)  = [kf.xk;  diag(kf.Pxk);  kf.res];
        akf = akfupdate(akf, Zkk(k,:)', 'B', 'MSHAKF',bs);       ares(k,:) = [akf.xk; diag(akf.Pxk); akf.res];
        mkf = akfupdate(mkf, Zkk(k,:)', 'B', 'MCKF',  5);       mres(k,:) = [mkf.xk; diag(mkf.Pxk); mkf.res];
        rkf = akfupdate(rkf, Zkk(k,:)', 'B', 'RSTKF', 3);       rres(k,:) = [rkf.xk; diag(rkf.Pxk); rkf.res];
        skf = akfupdate(skf, Zkk(k,:)', 'B', 'SSMKF', 3);       sres(k,:) = [skf.xk; diag(skf.Pxk); skf.res];
        ckf = akfupdate(ckf, Zkk(k,:)', 'B', 'CERKF', 3);       cres(k,:) = [ckf.xk; diag(ckf.Pxk); ckf.res];
    end
    sp = sp + delbias([res(:,1),ares(:,1),rres(:,1),mres(:,1),sres(:,1),cres(:,1)], Xkk(:,1)).^2;
    sv = sv + delbias([res(:,2),ares(:,2),rres(:,2),mres(:,2),sres(:,2),cres(:,2)], Xkk(:,2)).^2;
    disp(MCn);
end
sp = sp/MCn;  sv = sv/MCn;
rmsepv = sqrt([mean(sp(100:end,:))', mean(sv(100:end,:))']);
lgstr = {'IdealKF','MSHAKF','RSTKF','MCKF','SSMKF','CERKF'};
myfig
subplot(221), plot([Xkk(:,1),Zkk(:,1)]); xygo('\itk','距离 / m'); legend('真实值', '测量值'); title('(a)')
subplot(222), plot([Xkk(:,2)]); xygo('\itk','速度 / m/s');  title('(b)')
subplot(223), plot(smoothn(sqrt(sp),10)); xygo('\itk','距离估计 RMSE / m');  title('(a)'); mylegend(lgstr,rmsepv(:,1));
subplot(224), plot(smoothn(sqrt(sv),10)); xygo('\itk','速度估计 RMSE / m');  title('(b)'); mylegend(lgstr,rmsepv(:,2));
return;
%%
myfig;
subplot(221), plot(Zkk(:,1)-Xkk(:,1)); xygo('k','dist meas err / m');
subplot(223), plot([Xkk(:,2),res(:,2),ares(:,2),mres(:,2),rres(:,2),sres(:,2),cres(:,2)]); xygo('k','vel / m/s'); 
    legend('real', 'IdealKF est','MSHAKF est','MCKF est','RSTKF est','SSMKF est','CERKF est');
    perr = [res(:,1)-Xkk(:,1),ares(:,1)-Xkk(:,1),mres(:,1)-Xkk(:,1),rres(:,1)-Xkk(:,1),sres(:,1)-Xkk(:,1),cres(:,1)-Xkk(:,1)];
subplot(222), plot(perr); xygo('k','dist RMSE / m'); plot(sqrt([res(:,3),ares(:,3),mres(:,3),rres(:,3),sres(:,3)]));
    perr = rms(perr(100:end,:)); mylegend(lgstr,perr);
    verr = [res(:,2)-Xkk(:,2),ares(:,2)-Xkk(:,2),mres(:,2)-Xkk(:,2),rres(:,2)-Xkk(:,2),sres(:,2)-Xkk(:,2),cres(:,2)-Xkk(:,2)];
subplot(224), plot(verr); xygo('k','vel RMSE / m/s'); plot(sqrt([res(:,4),ares(:,4),mres(:,4),rres(:,4),sres(:,4)])); 
    verr = rms(verr(100:end,:)); mylegend(lgstr,verr);
myfig;
subplot(221), xygo('k','dist RMSE / m'); plot(sqrt([res(:,3),ares(:,3),mres(:,3),rres(:,3),sres(:,3)]));    legend(lgstr)
subplot(222), xygo('k','vel RMSE / m/s'); plot(sqrt([res(:,4),ares(:,4),mres(:,4),rres(:,4),sres(:,4)]));   legend(lgstr);
subplot(2,2,[3,4]); plot([abs(wnk), res(:,5),ares(:,5),mres(:,5),rres(:,5),sres(:,5),cres(:,5)]), xygo('k','val'); legend('wn', 'KF','MSHAKF','MCKF','RSTKF','SSMKF','CERKF');

