function kf = kfsetting(user_case, nts)
% SINS/GPS Kalman filter setting 15->34 states, 3/6 measurements.
%
% Prototype: kf = kfsetting(user_case)
% Input: user_case - user case define
% Output: kf - Kalman filter structure array
%
% See also  kfinit, kfinit0, psinstypedef.
%
% Example:
%    glvs; psinstypedef(193);
%    kf = kfsetting(0);

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 13/03/2021
global glv psinsdef
% phi(3), dvn(3), dpos(3), eb(3), db(3), lever(3), dT(1), dKg(9), dKa(6), (total states 6*3+1+9+6=34)
    if nargin<2, nts=0.01; end
    if nargin<1, user_case=0; end
    if ischar(user_case),
        [Qt,Pk,Pmax,Pmin,Rk,Rmax,Rmin] = feval(user_case); user_case=0;
    end
    switch(user_case)
        case 0,  % inertial grade
    Qt = [0.001,0.001,0.001, 1,1,1, zeros(1,28)];
    Pk = [1,1,3, 1,1,1, 10,10,10, 0.01,0.01,0.01, 100,100,100,    1,1,1, 0.1, ...
        10,10,10, 10,10,10, 10,10,10,   10,10,10, 10,10, 10];
    Pmax = [10,10,30, 100,100,100, 1e5,1e5,1e5, 0.5,0.5,0.5, 10000,10000,10000,    10,10,10, 0.5, ...
        1000,1000,1000, 1000,1000,1000, 1000,1000,1000,   1000,1000,1000, 1000,1000, 1000];
    Pmin = [0.001,0.001,0.003, 0.001,0.001,0.001, 0.001,0.001,0.001, 0.001,0.001,0.001, 1,1,10,   0.01,0.01,0.01, 0.001, ...
        1,1,1, 1,1,1, 1,1,1,   1,1,1, 1,1, 1];
	Rk = [0.1,0.1,0.1, 10,10,10];
    Rmax = [1,1,1, 100,10,100];
    Rmin = [0.01,0.01,0.01, 0.01,0.01,0.01];
        case 1,  % stim300 grade
    Qt = [0.1,0.1,0.1, 10,10,10, zeros(1,28)];
    Pk = [1,1,10, 1,1,1, 10,10,10, 100,100,100, 3000,3000,3000,    1,1,1, 0.1, ...
        10,10,10, 10,10,10, 10,10,10,   10,10,10, 10,10, 10];
    Pmax = [10,10,30, 100,100,100, 1e5,1e5,1e5, 1000,1000,1000, 10000,10000,10000,    10,10,10, 0.5, ...
        1000,1000,1000, 1000,1000,1000, 1000,1000,1000,   1000,1000,1000, 1000,1000, 1000];
    Pmin = [0.01,0.01,0.03, 0.001,0.001,0.001, 0.001,0.001,0.001, 0.1,0.1,0.1, 30,30,30,   0.01,0.01,0.01, 0.001, ...
        1,1,1, 1,1,1, 1,1,1,   1,1,1, 1,1, 1];
	Rk = [0.1,0.1,0.1, 10,10,10];
    Rmax = [1,1,1, 100,10,100];
    Rmin = [0.01,0.01,0.01, 0.01,0.01,0.01];
    end
    
    Qt(1:3) = Qt(1:3)*glv.dpsh; Qt(4:6) = Qt(4:6)*glv.ugpsHz;
    Pk = Pcvt(Pk);  Pmax = Pcvt(Pmax);  Pmin = Pcvt(Pmin);
    Rk(4:5) = Rk(4:5)/glv.Re;  Rmax(4:5) = Rmax(4:5)/glv.Re;  Rmin(4:5) = Rmin(4:5)/glv.Re;
    nq = 1:psinsdef.kffk;
    kf.Qt = diag(Qt(nq).^2);
    kf.Pxk = diag(Pk(nq).^2);
    nr = 1:(psinsdef.kfhk-10*psinsdef.kffk); if length(nr)<6, nr=nr+3; end
    kf.Rk = diag(Rk(nr).^2);  kf.Rmax = diag(Rmax(nr).^2);  kf.Rmin = diag(Rmin(nr).^2);
    kf.Hk = zeros(nr(end),nq(end));
    kf = kfinit0(kf, nts);
    kf.Pmax = Pmax(nq).^2;  kf.Pmin = Pmin(nq).^2;
    kf.adaptive = 1;
    kf.pconstrain = 1;

function P = Pcvt(P)
global glv
    P(1:3)=P(1:3)*glv.deg;  P(7:8)=P(7:8)/glv.Re;
    P(10:12)=P(10:12)*glv.dph;  P(13:15)=P(13:15)*glv.ug;
    P([20,24,28, 29,32,34])=P([20,24,28, 29,32,34])*glv.ppm;
    P([21:23,25:27, 30,31,33])=P([21:23,25:27, 30,31,33])*glv.sec;
