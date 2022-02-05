function [Fk, Ft] = ekffk(ins, afa)
% Create Kalman filter system transition matrix.
%
% Prototype: [Fk, Ft] = ekffk(ins, afa)
% Inputs: ins - SINS structure array from function 'insinit'
%         afa - = states Xk(1:3), for large azimuth misalignment angle
% Outputs: Fk - discrete-time transition matrix
%          Ft - continuous-time transition matirx
%
% See also  kffk, Jacob5, ekf.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/08/2012, 01/02/2014
    [Fk, Ft] = kffk(ins);
    sa = sin(afa); ca = cos(afa);
    ax = afa(1); ay = afa(2); saz = sa(3); caz = ca(3);
    wE = ins.eth.wnin(1); wN = ins.eth.wnin(2); wU = ins.eth.wnin(3);
    fnx = ins.fn(1); fny = ins.fn(2); fnz = ins.fn(3); 
    jF = zeros(6,3);
    jF(1,2) = wU; jF(1,3) = wE*saz-caz*wN; 
    jF(2,1) = -wU; jF(2,3) = wE*caz+saz*wN;
    jF(3,1) = -wE*saz+caz*wN; jF(3,2) = -wE; jF(3,3) = -ax*(wE*caz+saz*wN); 
    jF(4,1) = -saz*fnz; jF(4,2) = -caz*fnz; jF(4,3) = saz*fnx+caz*fny-(-ay*saz+ax*caz)*fnz;
    jF(5,1) = caz*fnz;  jF(5,2) = -saz*fnz; jF(5,3) = -caz*fnx+saz*fny-(ay*caz+ax*saz)*fnz;
    jF(6,1) = -fny;     jF(6,2) = -fnx;
    Fk(1:6,1:3) = [eye(3)+jF(1:3,:)*ins.nts; jF(4:6,:)*ins.nts];
    