% The precession & nutation motion simulation for two degree-of-freedom gyro.
% Solution of the following differential equations:
%       J*dotdot_thetaX + H*dot_thetaY = Mx
%       J*dotdot_thetaY - H*dot_thetaX = My
% See also demo_gyro_rotor_precession.
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/09/2022
glvs;
Ts = 0.00001;
len = fix(10/Ts);
t = (1:len)'*Ts;
thetaX = zeros(len,1); thetaY = thetaX;
J = 0.1*0.01^2;  % kg*m^2;
H = 2*pi*400*J;
Mx = 0; My = 0;  M = 1e-3*9.8*0.01; % N*m
for k=3:len
    if 1/Ts<k && k<2/Ts, Mx=M; else Mx=0; end
    thetaXk_2 = thetaX(k-2); thetaXk_1 = thetaX(k-1);  thetaYk_2 = thetaY(k-2); thetaYk_1 = thetaY(k-1);
    thetaXk = (My-H*(thetaYk_1-thetaYk_2)/Ts) / J * Ts^2   - thetaXk_2 + 2*thetaXk_1;
    thetaYk   = (Mx+H*(thetaXk-thetaXk_1)/Ts) / J * Ts^2 - thetaYk_2 + 2*thetaYk_1;
    thetaX(k) = thetaXk; thetaY(k) = thetaYk;
end
myfig, plot(t, [thetaX, thetaY]/glv.min);  xygo('Prcession & Nutation / ( \prime )')
