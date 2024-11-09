function [imu, ts, t0, bias, scale, mami] = imuzip(imu, sg, sa, ts, t0)
% IMU double-data zip to 1/2 bytes array, or reverse.
%
% Prototype: [imu, ts, t0, bias, scale, mami] = imuzip(imu, sg, sa, ts, t0)
% Inputs: imu - IMU data input
%         sg - scale for gyro, [1/128,127]*arcsec, default 1 arcsec
%         sa - scale for acc, [1/128,127]*100ug*s, default 100ug*s
%         ts - sample interval, [1,127*127*127/1000=2048Hz]
%         t0 - IMU start time, [-38d,127^5*0.0001=3.3e6sec=38d]
% Outputs: imu, ts, t0 - as input
%          bias - [-128,127]*(0.1*dps),(0.01g)
%          scale - [1/128,127]*(arcsec),(100ug*s)
%          mami - max & min IMU interger value
%
% Example:
%    imu0 = imustatic(avpset([10;20;30],0,glv.pos0), 0.01, 100, imuerrset(10000,1000,1,10));
%    [imu1, ts, t0, bias, scale, mm] = imuzip(imu0, 1/2, 1/2, 0.01, -1000);
%    [imu2, ts, t0, bias, scale] = imuzip(imu1);  imuplot(adddt(imu0,t0-ts),imu2);
%    plotn(cumsum(imu2-adddt(imu0,t0-ts)));
%
% See also  imufile.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/11/2024
    arcdeg = pi/180; arcsec = arcdeg/3600;  g0 = 9.78;  dps = arcdeg/1; g0s = g0*1;
    if ~isinteger(imu) % zip
        if nargin<5, t0 = imu(1,end); end
        if nargin<4, ts = (imu(end,end)-imu(1,end))/(length(imu)-1); end
        frq001 = fix(1000/ts);  ts = 1000/frq001;  imu=imu(:,1:6);
        if nargin<3, sa = 1; end;  sa0=sa; if sa<1, sa=-1/sa; end
        if nargin<2, sg = 1; end;  sg0=sg; if sg<1, sg=-1/sg; end
        scale=[[1,1,1]*sg0*arcsec, [1,1,1]*sa0*0.0001*g0s];
        mn = mean(imu);
        bias = fix([mn(1:3)/(0.1*dps*ts), mn(4:6)/(0.01*g0*ts)]);
        b = [bias(1:3)*(0.1*dps*ts), bias(4:6)*(0.01*g0*ts)];
        for k=1:6, imu(:,k)=imu(:,k)-b(k); end
        imu = cumsum(imu);
        for k=1:6, imu(:,k) = imu(:,k)/scale(k); end; scale=scale([1,4]);
        imu = diff([zeros(1,6,'int64'); int64(imu(:,1:6))]);
        s=0; t=t0; if t<0, t=-t; s=-1; end
        t=fix(t/0.0001); t06=mod(t,127); t=fix(t/127); t05=mod(t,127); t=fix(t/127); t04=mod(t,127); t=fix(t/127);
                         t03=mod(t,127); t=fix(t/127); t02=mod(t,127); t=fix(t/127); t01=mod(t,127);
        % 1st row [scale_gyro(arcsec), scale_acc(100ug), fs(0.001Hz,3bytes)]
        % 2st row [t0(0.1ms,6bytes)]
        % 3nd row [bias_gx, bias_gy, bias_gz,  bias_ax, bias_ay, bias_az]
        f = frq001; f3 = mod(f,127); f = fix(f/127);
        i1 = int16([sg, sa, fix(f/127), mod(f,127), f3,0]);
        i2 = int16([s*128+t01, t02, t03, t04, t05, t06]);
        i3 = int16(bias);
        mami = [max(max(imu)), min(min(imu))];
        if isempty(find(bias>127|bias<-128,1)) && mami(1)<=127 && mami(2)>=-128
            imu = [int8([i1; i2; i3]); int8(imu)];
        else
            imu = [[i1; i2; i3]; int16(imu)];
        end
    else % upzip
        % 1st row [scale_gyro(arcsec), scale_acc(100ug), fs(0.001Hz,3bytes)]
        % 2st row [t0(0.1ms,6bytes)]
        % 3nd row [bias_gx, bias_gy, bias_gz,  bias_ax, bias_ay, bias_az]
        i1 = double(imu(1,:));
        sg=i1(1); if sg<0, sg=-1/sg; end;
        sa=i1(2); if sa<0, sa=-1/sa; end;
        scale=[[1,1,1]*sg*arcsec, [1,1,1]*sa*0.0001*g0s];
        ts=1000/(i1(3)*127^2+i1(4)*127+i1(5));
        i2 = double(imu(2,:));
        s=1; if i2(1)<0, i2(1)=i2(1)+128; s=-1; end
        t0=s*(i2(1)*127^5+i2(2)*127^4+i2(3)*127^3+i2(4)*127^2+i2(5)*127^1+i2(6))*0.0001;  % 0.1ms
        i3 = double(imu(3,:));  b=[i3(1:3)*0.1*dps*ts, i3(4:6)*0.01*g0*ts];  bias=imu(3,:);
        imu = double(imu(4:end,:));
        for k=1:6, imu(:,k)=imu(:,k)*scale(k)+b(k); end; scale=scale([1,4]);
        imu = [imu,t0+(0:length(imu)-1)'*ts];
    end
