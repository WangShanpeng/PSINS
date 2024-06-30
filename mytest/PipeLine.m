function [posdr,posdr0,posdr1] = PipeLine(imuod, t, pos, yaw0, inst, kod, Td)
% 输入参数：
%   imuod = [gx,gy,gz, ax,ay,az, od, t];  gx,gy,gz 角增量/rad（右前上）; ax,ay,az 速度增量/(m/s); od 里程计距离增量/m; t 时间/s
%   t = [t0, t1, t2];  t0 起始时间，确保之后10s静止;  t1 往程末端时间，确保前后20s都静止;  t2 回程末了时间
%   pos = [pos0; pos1]  pos0 起始位置（纬经rad，高m）; pos1 往程末端位置
%   yaw0 初始方位角/rad 北偏西为正 -pi -> pi
%   inst 惯导安装偏差 [俯仰偏差; 0; 方位偏差]/rad,  默认为 [0;0;0]
%   kod 里程刻度系数（m/脉冲）,  默认为 1
%   Td 调平时间常数,  默认为 0
% 输出参数：
%   posdr = 往返程融合位置（纬经rad，高m）
%   posdr0,posdr1 = 往、返程位置
    t0 = t(1); t1 = t(2); t2 = t(3);
    pos0 = pos(1,1:3)';  pos1 = pos(2,1:3)';
    if nargin<7, Td=0; end
    if nargin<6, kod=1; end
    if nargin<5, inst=zeros(3,1); end
    glvwie(0);
    %%
    imuod = imuresample(imurepair(imuod),0.01,0,'spline',0);  % imuplot(imuod(:,[1:7,8]),1); odplot(imuod(:,7:8));
    att = alignsb(datacut(imuod,t0,t0+10),pos0); att(3) = yaw0;
    imuod(:,[1:6,8]) = imudeldrift(imuod(:,[1:6,8]), t1-10,t1+10);
    avp = drpure(datacut(imuod,t0,t2), [att;pos0], inst, kod, Td); close(gcf); % insplot(avp);
    pos1DR = getat(avp,t1);
    [inst1, kod] = drcalibrate(avp(1,7:9)', pos1, pos1DR(7:9));
    avp = drpure(datacut(imuod,t0,t2), [att-inst1;pos0], inst, kod, Td); % insplot(avp);
    %%
    posdr0 = drfit(datacut(avp(:,[7:9,end]),1,t1), pos0, pos1, 1);  title('forward fit');
    posdr1 = drfit(datacut(avp(:,[7:9,end]),t1,t2), pos1, pos0, 1);  title('backward fit');
    posdr = drfusion(posdr0, posdr1, 1, 2); 
    glvwie(1);



