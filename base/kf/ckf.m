function kf = ckf(kf, yk)
% Cubature transformation KF.
%
% Prototype: kf = ckf(kf, yk)
% Inputs: kf - filter structure array
%         yk - measurement vector
% Output: kf - filter structure array after time/meas updating
%
% See also  ckfUT, ukf, ekf, kfupdate.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/03/2022
    if isfield(kf, 'fx')  % nonliear state propagation
        [kf.xkk_1, kf.Pxkk_1] = ckfCT(kf.xk, kf.Pxk, kf.fx, kf.px);
        kf.Pxkk_1 = kf.Pxkk_1 + kf.Gammak*kf.Qk*kf.Gammak';
    else
        kf.xkk_1 = kf.Phikk_1*kf.xk;
        kf.Pxkk_1 = kf.Phikk_1*kf.Pxk*kf.Phikk_1' + kf.Gammak*kf.Qk*kf.Gammak';
    end
    if nargin<2    % time updating only
        kf.xk = kf.xkk_1; kf.Pxk = kf.Pxkk_1;
        return;
    end
    
    if isfield(kf, 'hx')  % nonliear measurement propagation
        [kf.ykk_1, kf.Pykk_1, kf.Pxykk_1] = ckfCT(kf.xkk_1, kf.Pxkk_1, kf.hfx, kf.py);
        kf.Pykk_1 = kf.Pykk_1 + kf.Rk;
    else
        kf.ykk_1 = kf.Hk*kf.xkk_1;
        kf.Pxykk_1 = kf.Pxkk_1*kf.Hk';    kf.Pykk_1 = kf.Hk*kf.Pxykk_1 + kf.Rk;
    end
    % filtering
    kf.Kk = kf.Pxykk_1*kf.Pykk_1^-1;
    kf.xk = kf.xkk_1 + kf.Kk*(yk-kf.ykk_1);
    kf.Pxk = kf.Pxkk_1 - kf.Kk*kf.Pykk_1*kf.Kk';  kf.Pxk = (kf.Pxk+kf.Pxk')/2;
