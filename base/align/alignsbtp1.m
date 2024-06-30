function [att2, att1, eb, db, phi, dyaw] = alignsbtp1(imu, pos, T1T2, Y1Y2)
% SINS align on static base using two-position method.
%
% Prototype: [att2, att1, eb, db, phi, dyaw] = alignsbtp1(imu, pos, T1T2, Y1Y2)
% Inputs: imu - SIMU data
%         pos - initial position
%         T1T2 - [-inf,T1T2(1)]/[T1T2(2),inf] for first/second angular position
%         Y1Y2 - from first to second angular rotation
% Outputs: att2, att1 - attitude align results Euler angles in second/first angular position
%          eb, db - gyro drift, acc bias
%
% Example:
%    imuerr = imuerrset([1;2;5]*0.01, [1;2;3]*100.0, 0, 0, 0,0,0,0, .100,0,100,0);
%    [imu1, att0] = imustatictp([[1;2;10]*glv.deg; glv.pos0], 0.1, 300, 10*glv.dps, 180*glv.deg, imuerr);
%    [att, eb, db, phi] = alignsbtp1(imu1, glv.pos0, [130,170], 0);  eb/glv.dph, db/glv.ug, phi/glv.min
%    insplot(att0, 'a'); plot(att0(end,1), att(3)/glv.deg, 'or');
%
% See also  alignsbtp, alignvn, aa2dkg, alignvntp.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/11/2022
    if nargin<4, Y1Y2=0; end
    ts = diff(imu(1:2,end));
    if length(pos)==1,  pos = [pos; 0; 0];   end
    eth = earth(pos);
    imu1 = datacut(imu,-inf,T1T2(1));    imu12 = datacut(imu,T1T2(1),T1T2(2));    imu2 = datacut(imu,T1T2(2),inf);
    att1 = alignsb(imu1, pos, 0);
	avp = inspure(imu12, [att1; pos], 'v', 0);  % insplot(avp);
	att2 = avp(end,1:3)';  dyaw = att2(3)-att1(3);
    if abs(Y1Y2)>pi/2, att2(3)=att1(3)+Y1Y2; end 
    Cnb1 = a2mat(att1); Cnb2 = a2mat(att2);
    wf1 = mean(imu1(:,1:6))'/ts; wf2 = mean(imu2(:,1:6))'/ts;
    Z =  [ Cnb1*wf1(1:3)-eth.wnie; Cnb2*wf2(1:3)-eth.wnie;
           Cnb1*wf1(4:6)+eth.gn;   Cnb2*wf2(4:6)+eth.gn ];
    O33 = zeros(3);
    H = [ askew(Cnb1*wf1(1:3)),  Cnb1, O33
          askew(Cnb2*wf2(1:3)),  Cnb2, O33
          askew(Cnb1*wf1(4:6)),  O33,  Cnb1
          askew(Cnb2*wf2(4:6)),  O33,  Cnb2 ];
    X = lscov(H, Z);
    X = lscov(H(:,[1:2,4:end]), Z);  X = [X(1:2);0;X(3:end)];
    phi = X(1:3); eb = X(4:6); db = X(7:9);
%     att2 = q2att(qdelphi(a2qua(att2),-phi));  % lower accuracy
    wf = wf2 - [eb;db];
    [~, att2] = dv2atti(eth.wnie, -eth.gn, wf(1:3), wf(4:6));
    wf = wf1 - [eb;db];
    [~, att1] = dv2atti(eth.wnie, -eth.gn, wf(1:3), wf(4:6));

    
