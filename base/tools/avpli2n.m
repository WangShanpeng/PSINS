function avpn = avpli2n(pos0, yaw0, avpi, t)
% Translate AVP from F-U-R launch-inertial frame to E-N-U local-level navigation frame.
%
% Prototype: avpn = avpli2n(pos0, yaw0, avpi, t)
% inputs: pos0 - launch position lat/lon/hgt, in rad/rad/m
%         yaw0 - launch initial yaw angle, positive for counter-clockwise
%         avpi - =[att;vel;pos] in launch-inertial frame,
%              att=[pitch;roll;yaw] in Pitch/Yaw/Roll rotation sequence;
%              vel=[Vx-front;Vy-up;Vz-right], in m/s;
%              pos=[X-front;Y-up;Z-right], in meter
%         t - avpi at launch epoch time t
% output: avpn=[Pitch;Roll;Yaw; VE;VN;VU; lat;lon;hgt], where Euler angles in
%              Yaw/Pitch/Roll rotation sequence, and lat/lon/hgt in rad/rad/m. 
%
% Example:
%    glvs;
%    avpn = avpli2n([[34;106]*glv.deg;380], -45*glv.deg, [[60;0;0]*glv.deg;[1000;500;0];[20000;10000;0]], 30);
%
% See also a2qua1, xyz2blh

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/02/2021
    eth = earth(pos0);
    Cny0 = rv2m(-eth.wnie*t)*a2mat([0;0;yaw0]);
    rn = Cny0*avpi([9,7,8]);
    [xyz0, Cen] = blh2xyz(pos0);
    xyz = xyz0+Cen*rn;
    pos = xyz2blh(xyz);
    vn = Cny0*avpi([6,4,5]);
    [~,Cy0b] = a2qua1(avpi(1:3));
    att = m2att(Cny0*Cy0b);
    avpn = [att; vn; pos];
    