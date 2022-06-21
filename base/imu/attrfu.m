function att1 = attrfu(att0, dirstr)
% Attitude / Euler angles transformation.
%
% Prototype: att1 = attrfu(att0, dirstr)
% Inputs: att0 - IMU mounted in 'dirstr', the IMU Euler angle outputs is att0.
%         dirstr - IMU X-Y-Z orientations including three characters, 
%               the orientation abbreviations are:
%               'U': Upper; 'D': Down; 'R': Right; 'L': Left; 
%               'F': Forword; 'B': Back.
% Output: att1 - Euler angle outputs for R-F-U vehicle.
%
% Example:
%   att1 = attrfu([0,0,0], 'flu')
%
% See also  imurfu, Cbbstr, wierfu, axxx2a.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/07/2021
    Cnb0 = a2matBatch(att0);
    Cb0b1 = Cbbstr('rfu', dirstr);
%     Cnb1 = [ Cnb0(1)*Cb0b1(1,1)+Cnb0(2)*Cb0b1(2,1)+Cnb0(3)*Cb0b1(3,1), ...
%              Cnb0(1)*Cb0b1(1,2)+Cnb0(2)*Cb0b1(2,2)+Cnb0(3)*Cb0b1(3,2), ...
%              Cnb0(1)*Cb0b1(1,3)+Cnb0(2)*Cb0b1(2,3)+Cnb0(3)*Cb0b1(3,3), ...
%              Cnb0(4)*Cb0b1(1,1)+Cnb0(5)*Cb0b1(2,1)+Cnb0(5)*Cb0b1(3,1), ...
%              Cnb0(4)*Cb0b1(1,2)+Cnb0(5)*Cb0b1(2,2)+Cnb0(5)*Cb0b1(3,2), ...
%              Cnb0(4)*Cb0b1(1,3)+Cnb0(5)*Cb0b1(2,3)+Cnb0(5)*Cb0b1(3,3), ...
%              Cnb0(7)*Cb0b1(1,1)+Cnb0(8)*Cb0b1(2,1)+Cnb0(9)*Cb0b1(3,1), ...
%              Cnb0(7)*Cb0b1(1,2)+Cnb0(8)*Cb0b1(2,2)+Cnb0(9)*Cb0b1(3,2), ...
%              Cnb0(7)*Cb0b1(1,3)+Cnb0(8)*Cb0b1(2,3)+Cnb0(9)*Cb0b1(3,3) ];
%     att = [ asin(Cnb(3,2));
%             atan2(-Cnb(3,1),Cnb(3,3)); 
%             atan2(-Cnb(1,2),Cnb(2,2)) ];
    Cnb32 = Cnb0(:,7)*Cb0b1(1,2)+Cnb0(:,8)*Cb0b1(2,2)+Cnb0(:,9)*Cb0b1(3,2);
    Cnb31 = Cnb0(:,7)*Cb0b1(1,1)+Cnb0(:,8)*Cb0b1(2,1)+Cnb0(:,9)*Cb0b1(3,1);
    Cnb33 = Cnb0(:,7)*Cb0b1(1,3)+Cnb0(:,8)*Cb0b1(2,3)+Cnb0(:,9)*Cb0b1(3,3);
    Cnb12 = Cnb0(:,1)*Cb0b1(1,2)+Cnb0(:,2)*Cb0b1(2,2)+Cnb0(:,3)*Cb0b1(3,2);
    Cnb22 = Cnb0(:,4)*Cb0b1(1,2)+Cnb0(:,5)*Cb0b1(2,2)+Cnb0(:,5)*Cb0b1(3,2);
    att1 = att0;
    att1(:,1:3) = [ asin(Cnb32),  atan2(-Cnb31,Cnb33),  atan2(-Cnb12,Cnb22) ];


