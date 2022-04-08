function [Xk, Pxk, Xkk_1, Pxkk_1] = kalman(Phikk_1, Gammak, Qk, Xk_1, Pxk_1, Hk, Rk, Zk, s)
% A simple Kalman filter By Yan Gongmin
    if nargin<9, s=1; end
    Xkk_1 = Phikk_1*Xk_1;
    Pxkk_1 = Phikk_1*s*Pxk_1*Phikk_1' + Gammak*Qk*Gammak';
    Pxykk_1 = Pxkk_1*Hk';
    Pykk_1 = Hk*Pxykk_1 + Rk;
    Kk = Pxykk_1*Pykk_1^-1;
    Xk = Xkk_1 + Kk*(Zk-Hk*Xkk_1);
    Pxk = Pxkk_1 - Kk*Pykk_1*Kk';
    Pxk = (Pxk+Pxk')/2;
