function [Phikk_1, Xkk_1] = Jacob5(Xk_1, tpara)
% Ref. Yan's Post-doctoral research report Eq.(4-23).
% See also alignvn_ekf, test_align_ekf, test_Jacob5.
%
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/09/2013
    wnie = tpara.wnie; fn = tpara.fn; ts = tpara.ts;
    ax = Xk_1(1); ay = Xk_1(2); az = Xk_1(3); dvx = Xk_1(4); dvy = Xk_1(5);
    caz = cos(az); saz = sin(az);
    wN = wnie(2); wU = wnie(3); fnx = fn(1); fny = fn(2); fnz = fn(3);
    %% Jacobian Matrix
    jF = zeros(length(Xk_1));
    jF(1,2) = wU; jF(1,3) = -caz*wN; 
    jF(2,1) = -wU; jF(2,3) = saz*wN;
    jF(3,1) = caz*wN; jF(3,3) = -ax*saz*wN; 
%     jF(3,1) = caz*wN; jF(3,2) = -saz*wN; jF(3,3) = -ax*saz*wN-ay*caz*wN;
    jF(4,1) = -saz*fnz; jF(4,2) = -caz*fnz; jF(4,3) = saz*fnx+caz*fny-(-ay*saz+ax*caz)*fnz;
    jF(5,1) = caz*fnz;  jF(5,2) = -saz*fnz; jF(5,3) = -caz*fnx+saz*fny-(ay*caz+ax*saz)*fnz;
    Phikk_1 = eye(length(Xk_1))+jF*ts;
    %% 1st-order state updating
    if nargout>1
        Xkk_1 = Xk_1;
        Xkk_1(1) = ax + (-saz*wN+ay*wU)*ts;
        Xkk_1(2) = ay + ((1-caz)*wN-ax*wU)*ts;
        Xkk_1(3) = az + ax*caz*wN*ts;   
    %     Xkk_1(3) = az + (ax*caz*wN-ay*saz*wN)*ts;
        Xkk_1(4) = dvx + ((1-caz)*fnx+saz*fny-(ay*caz+ax*saz)*fnz)*ts;
        Xkk_1(5) = dvy + (-saz*fnx+(1-caz)*fny-(ay*saz-ax*caz)*fnz)*ts;
%         Xkk_1 = Phikk_1*Xk_1;       % error ! 2022-7-23
    end
