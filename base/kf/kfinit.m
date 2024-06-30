function kf = kfinit(ins, varargin)
% Kalman filter initializes for structure array 'kf', this precedure 
% usually includs the setting of structure fields: Qt, Rk, Pxk, Hk.
%
% Prototype: kf = kfinit(ins, varargin)
% Inputs: ins - SINS structure array, if not struct then nts=ins;
%         varargin - if any other parameters
% Output: kf - Kalman filter structure array
%
% See also  kfinit0, kfsetting, kffk, kfkk, kfupdate, kffeedback, psinstypedef.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/10/2013
global glv psinsdef
[Re,deg,dph,ug,mg] = ... % just for short
    setvals(glv.Re,glv.deg,glv.dph,glv.ug,glv.mg); 
o33 = zeros(3); I33 = eye(3); 
kf = [];
if isstruct(ins),    nts = ins.nts;
else                 nts = ins;
end
switch(psinsdef.kfinit)
    case psinsdef.kfinit153,
        psinsdef.kffk = 15;  psinsdef.kfhk = 153;  psinsdef.kfplot = 15;
        [davp, imuerr, rk] = setvals(varargin);
        kf.Qt = diag([imuerr.web; imuerr.wdb; zeros(9,1)])^2;
        kf.Rk = diag(rk)^2;
        kf.Pxk = diag([davp; imuerr.eb; imuerr.db]*1.0)^2;
        kf.Hk = kfhk(0);
    case psinsdef.kfinit156,
        psinsdef.kffk = 15;  psinsdef.kfhk = 156;  psinsdef.kfplot = 15;
        [davp, imuerr, rk] = setvals(varargin);
        kf.Qt = diag([imuerr.web; imuerr.wdb; zeros(9,1)])^2;
        kf.Rk = diag(rk)^2;
        kf.Pxk = diag([davp; imuerr.eb; imuerr.db]*1.0)^2;
        kf.Hk = kfhk(0);
    case psinsdef.kfinit183,
        psinsdef.kffk = 18;  psinsdef.kfhk = 183;  psinsdef.kfplot = 18;
        [davp, imuerr, lever, r0] = setvals(varargin);
        kf.Qt = diag([imuerr.web; imuerr.wdb; zeros(9+3,1)])^2;
        kf.Rk = diag(r0)^2;
        kf.Pxk = diag([davp; imuerr.eb; imuerr.db; lever]*1.0)^2;
        kf.Hk = zeros(3,18);
    case psinsdef.kfinit186,
        psinsdef.kffk = 18;  psinsdef.kfhk = 186;  psinsdef.kfplot = 18;
        [davp, imuerr, lever, r0] = setvals(varargin);
        kf.Qt = diag([imuerr.web; imuerr.wdb; zeros(3,1); imuerr.sqg; imuerr.sqa; zeros(3,1)])^2;
        kf.Rk = diag(r0)^2;
        kf.Pxk = diag([davp; imuerr.eb; imuerr.db; lever]*1.0)^2;
        kf.Hk = zeros(6,18);
    case psinsdef.kfinit193
        psinsdef.kffk = 19;  psinsdef.kfhk = 193;  psinsdef.kfplot = 19;
        [davp, imuerr, lever, dT, r0] = setvals(varargin);
        kf.Qt = diag([imuerr.web; imuerr.wdb; [1/Re;1/Re;1]*glv.mpsh; ...
            [1;1;1]*0*glv.dphpsh; [1;1;1]*0*glv.ugpsh; [1;1;1]*0.*glv.mpsh; 0])^2;
        kf.Rk = diag(r0)^2;
        kf.Pxk = diag([davp; imuerr.eb; imuerr.db; lever; dT]*1.0)^2;
        kf.Hk = zeros(3,19);
    case psinsdef.kfinit196
        psinsdef.kffk = 19;  psinsdef.kfhk = 196;  psinsdef.kfplot = 19;
        [davp, imuerr, lever, dT, r0] = setvals(varargin);
        kf.Qt = diag([imuerr.web; imuerr.wdb; [1/Re;1/Re;1]*0*glv.mpsh; ...
            [1;1;1]*0*glv.dphpsh; [1;1;1]*0*glv.ugpsh; [1;1;1]*0*glv.mpsh; 0])^2;
        kf.Rk = diag(r0)^2;
        kf.Pxk = diag([davp; imuerr.eb; imuerr.db; lever; dT]*1.0)^2;
        kf.Hk = zeros(6,19);
    case psinsdef.kfinit246
        psinsdef.kffk = 24;  psinsdef.kfhk = 246;  psinsdef.kfplot = 24;
        [davp, imuerr, r0] = setvals(varargin);
        kf.Qt = diag([imuerr.web; imuerr.wdb; [1/Re;1/Re;1]*0*glv.mpsh; ...
            [1;1;1]*0*glv.dphpsh; [1;1;1]*0*glv.ugpsh; zeros(9,1)])^2;
        kf.Rk = diag(r0)^2;
        kf.Pxk = diag([davp; imuerr.eb; imuerr.db; imuerr.dKga(1:9)]*1.0)^2;
        kf.Hk = zeros(6,24);
    case psinsdef.kfinit331,
        psinsdef.kffk = 33;  psinsdef.kfhk = 331;  psinsdef.kfplot = 33;
        [davp, imuerr, r0] = setvals(varargin);
        kf.Qt = diag([imuerr.web; imuerr.wdb; zeros(9+15+3,1)])^2;
        kf.Rk = diag(r0)^2;
        kf.Pxk = diag([davp; imuerr.eb; imuerr.db; imuerr.dKga; imuerr.Ka2])^2;
        kf.Hk = kfhk(ins);
        kf.xtau(1:psinsdef.kffk,1) = 0;
    case psinsdef.kfinit346,
        psinsdef.kffk = 34;  psinsdef.kfhk = 346;  psinsdef.kfplot = 34;
        [davp, imuerr, lever, dT, r0] = setvals(varargin);
        kf.Qt = diag([imuerr.web; imuerr.wdb; zeros(9+3+1+15,1)])^2;
        kf.Rk = diag(r0)^2;
        kf.Pxk = diag([davp; imuerr.eb; imuerr.db; lever; dT; imuerr.dKga])^2;
        kf.Hk = kfhk(ins);
        kf.xtau(1:psinsdef.kffk,1) = 0;
    case psinsdef.kfinit376,
        psinsdef.kffk = 37;  psinsdef.kfhk = 376;  psinsdef.kfplot = 37;
        [davp, imuerr, lever, dT, r0] = setvals(varargin);
        kf.Qt = diag([imuerr.web; imuerr.wdb; zeros(9+3+1+15+3,1)])^2;
        kf.Rk = diag(r0)^2;
        kf.Pxk = diag([davp; imuerr.eb; imuerr.db; lever; dT; imuerr.dKga; davp(4:6)]*10)^2;
        kf.Hk = kfhk(ins);
        kf.xtau(1:psinsdef.kffk,1) = 0;
    otherwise,
        kf = feval(psinsdef.typestr, psinsdef.kfinittag, [{ins},varargin]);
end
kf = kfinit0(kf, nts);
