% SINS Euler-platform-error-model propagation accuracy verification.
% Ref. Yan G, 'On SINS In-movement Initial Alignment and Some Other Problems',
%      Postdoctoral work report@NWPU, 2018.
% See also  test_SINS_error_model_verify.
% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 25/07/2020
glvs;
[nn, ts, nts] = nnts(1, 0.01);
avp0 = [[0;0;0]*glv.deg; [0;0;0]; [0;0]*glv.deg;0];  % init ture AVP
imuerr = imuerrset(0.1, 1000, 0, 0);
[imu, eth0] = imustatic(avp0, ts, 60, imuerr);  len = length(imu);
afa = [10; 80; -90]*glv.deg;  dvn = [0;100;0];  dpos = [[1;1]*glv.deg;100];  % large init AVP errors
ins = insinit([q2att(qaddafa(a2qua(avp0(1:3)),afa)); avp0(4:6)+dvn; avp0(7:9)+dpos], ts);
avperr1 = zeros(len,10); avperr2 = avperr1;
timebar(1, len);
for k=1:len
    % error type-1: error model propagation
    vE = ins.vn(1); vN = ins.vn(2); vU = ins.vn(3);        dvE = dvn(1); dvN = dvn(2); dvU = dvn(3);
    Lat = ins.pos(1); Lon = ins.pos(2); Hgt = ins.pos(3);  dLat = dpos(1); dLon = dpos(2); dHgt = dpos(3);
    ddLat = vN/ins.eth.RMh-(vN-dvN)/(ins.eth.RMh-dHgt);  % Eq.4-21a
    ddLon = vE*sec(Lat)/ins.eth.RNh-(vE-dvE)*sec(Lat-dLat)/(ins.eth.RNh-dHgt);  % Eq.4-21b
    ddHgt = dvU;  % Eq.4-21c
    dwnie = glv.wie*[0; cos(Lat)-cos(Lat-dLat); sin(Lat)-sin(Lat-dLat)];  % Eq.4-20a
    Lon_dot = vE*sec(Lat)/ins.eth.RNh;
    dwnen = [-ddLat; Lon_dot*cos(Lat)-(Lon_dot-ddLon)*cos(Lat-dLat); Lon_dot*sin(Lat)-(Lon_dot-ddLon)*sin(Lat-dLat)];  % Eq.4-20b
    Cnp = a2mat(afa);  % Eq.4-4
    ddvn = (eye(3)-Cnp)*qmulv(ins.qnb,imu(k,4:6)'/ts) + Cnp*(qmulv(ins.qnb,imuerr.db))  ...
            -cross((2*dwnie+dwnen),ins.vn-dvn) - cross((2*ins.eth.wnie+ins.eth.wnen),dvn) + (ins.eth.gn-eth0.gn);  % Eq.4-19
    Cw = a2cwa(afa);  % Eq.4-7
    dafa = Cw^-1*((eye(3)-Cnp')*(ins.eth.wnie+ins.eth.wnen) + Cnp'*(dwnie+dwnen) - qmulv(ins.qnb,imuerr.eb));  % Eq.4-17
    if k==1, dafa0=dafa; ddvn0=ddvn; ddLat0=ddLat; ddLon0=ddLon; ddHgt0=ddHgt; end
    afa = afa + (3/2*dafa-1/2*dafa0)*ts;  % extrapolating discretization
    dvn = dvn + (3/2*ddvn-1/2*ddvn0)*ts;  
    dpos = dpos + (3/2*[ddLat;ddLon;ddHgt]-1/2*[ddLat0;ddLon0;ddHgt0])*ts;
    dafa0=dafa; ddvn0=ddvn; ddLat0=ddLat; ddLon0=ddLon; ddHgt0=ddHgt;
    avperr1(k,:) = [afa; dvn; dpos; k*ts]';
    % error type-2: SINS algorithm updating error
    ins = insupdate(ins, imu(k,:));
    avperr2(k,:) = [qq2afa(ins.qnb,a2qua(avp0(1:3))); ins.vn-avp0(4:6); ins.pos-avp0(7:9); k*ts]';
    timebar;
end
avpcmpplot(avperr1, avperr2); % display the difference

