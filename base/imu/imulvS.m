function SS = imulvS(wb, dotwb, Cba)
% Ref:'Inner Lever Arm Compensation and Its Test Verification for SINS' Eq.(12).
%
% Prototype: SS = imulvS(wb, dotwb, Cba)
% Inputs: wb - gyro algular rate
%         dotwb - gyro angular acceleration
%         Cba - IMU body to acc sensor frame
% Output: SS - output lever related matrix
%
% See also  sysclbt, imuclbt.

% Copyright(c) 2009-2016, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 16/08/2016
    if nargin<3, Cba=eye(3); end
    U = (Cba')^-1; V1 = Cba(:,1)'; V2 = Cba(:,2)'; V3 = Cba(:,3)';
    Q11 = U(1,1)*V1; Q12 = U(1,2)*V2; Q13 = U(1,3)*V3;
    Q21 = U(2,1)*V1; Q22 = U(2,2)*V2; Q23 = U(2,3)*V3;
    Q31 = U(3,1)*V1; Q32 = U(3,2)*V2; Q33 = U(3,3)*V3;
    W = askew(dotwb)+askew(wb)^2;
    SS = [Q11*W, Q12*W, Q13*W; Q21*W, Q22*W, Q23*W; Q31*W, Q32*W, Q33*W];
