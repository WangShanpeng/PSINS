function [avp, jk] = nhcpure(imu, avp0, dyaw, accth, od)
% Process NHC(nonholonomic constraint) update for low-grade IMU in car motion.
%
% Prototype: [avp, jk] = nhcpure(imu, avp0, dyaw, vth)
% Inputs: imu - SIMU data array
%         avp0 - initial avp0=[att0,vn0,pos0]
%         dyaw - angle between IMU/car front directions.
%         accth - acc threshold.
%         
% Outputs: avp - avp navigation result
%          phi - attitude error
%
% See also  inspure, drpure, insinstant, insupdate.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/06/2021
    if nargin<3, dyaw=0; end
    if nargin<4, accth=0.2; end
    ts = diff(imu(1:2,end));
    qnb = a2qua(avp0(1:3)); vn = avp0(4:6); pos = avp0(7:9);
    eth = earth(pos);
    len = length(imu);
    avp = zeros(len, 10); jk = zeros(len, 4);
    timebar(1, len, 'NHC navigation processing.');
        an = qmulv(qnb, imu(1,4:6)'/ts) + eth.gcc;
        att = q2att(qnb);
        att1 = [0;0;att(3)+dyaw];
        Cno = a2mat(att1);
        ao = Cno'*an; aj = ao;
    for k=1:len
        an = (qmulv(qnb, imu(k,4:6)'/ts) + eth.gcc);
        qnb = qupdt2(qnb, imu(k,1:3)', eth.wnie*ts);  att = q2att(qnb);
        att1 = [0;0;att(3)+dyaw];
        Cno = a2mat(att1);
        ao = Cno'*an; aj = 0.95*aj+0.05*ao;
        aj(1)=0.95*aj(1);
        ajj=aj;
%         if norm(ajj)<accth, ajj=ajj*0;
        if norm(ajj(2))<accth, ajj(2)=0;
        end
        vn = vn + Cno*ajj*ts;
        vo = Cno'*vn;  vn = Cno*[0;od(k,1)+0*vo(2);0];
        jk(k,:) = [aj; ajj(2)]';
        pos = pos + vn2dpos(eth, vn, ts);
        avp(k,:) = [att; vn; pos; imu(k,end)];
        timebar;
    end

%     an = qmulv(qnb, imu(1,4:6)'/ts) + eth.gcc;
%     for k=1:len
%         an = 0.99*an + 0.01*(qmulv(qnb, imu(k,4:6)'/ts) + eth.gcc);
%         qnb = qupdt2(qnb, imu(k,1:3)', eth.wnie*ts);  att = q2att(qnb);
%         if norm(an(1:2))>0.2
%             vn = vn + an*ts;
%         end
%         att1 = [0;0;att(3)+dyaw];
%         Cno = a2mat(att1);
%         vo = Cno'*vn;  vn = Cno*[0;vo(2);0];
%         jk(k,:) = an';
%         pos = pos + vn2dpos(eth, vn, ts);
%         avp(k,:) = [att; vn; pos; imu(k,end)];
%         timebar;
%     end