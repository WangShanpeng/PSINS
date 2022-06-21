function kf = ssukf(kf, yk, TimeMeasBoth)
% Unscented Kalman filter for nonlinear system.
%
% Prototype: kf = ssukf(kf, yk, TimeMeasBoth)
% Inputs: kf - filter structure array
%         yk - measurement vector
%         TimeMeasBoth - described as follows,
%            TimeMeasBoth='T' (or nargin==1) for time updating only, 
%            TimeMeasBoth='M' for measurement updating only, 
%            TimeMeasBoth='B' (or nargin==2) for both time and 
%                             measurement updating.
% Output: kf - filter structure array after time/meas updating
%
% See also  ukf, SSUT, ckf, ekf, kfupdate.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/06/2022
    if nargin==1;
        TimeMeasBoth = 'T';
    elseif nargin==2
        TimeMeasBoth = 'B';
    end

    if TimeMeasBoth=='T' || TimeMeasBoth=='B'
        if isfield(kf, 'fx')  % nonliear state propagation
            [kf.xkk_1, kf.Pxkk_1] = SSUT(kf.xk, kf.Pxk, kf.fx, kf.px, 0);
            kf.Pxkk_1 = kf.Pxkk_1 + kf.Gammak*kf.Qk*kf.Gammak';
        else
            kf.xkk_1 = kf.Phikk_1*kf.xk;
            kf.Pxkk_1 = kf.Phikk_1*kf.Pxk*kf.Phikk_1' + kf.Gammak*kf.Qk*kf.Gammak';
        end
        if TimeMeasBoth=='T'    % time updating only
            kf.xk = kf.xkk_1; kf.Pxk = kf.Pxkk_1;
            return;
        end
    end
    
    if TimeMeasBoth=='M' || TimeMeasBoth=='B'
        if TimeMeasBoth=='M'    % meas updating only
            kf.xkk_1 = kf.xk; kf.Pxkk_1 = kf.Pxk;
        end
        if isfield(kf, 'hx')  % nonliear measurement propagation
            [kf.ykk_1, kf.Pykk_1, kf.Pxykk_1] = SSUT(kf.xkk_1, kf.Pxkk_1, kf.hx, kf.py, 0);
            kf.Pykk_1 = kf.Pykk_1 + kf.Rk;
        else
            kf.ykk_1 = kf.Hk*kf.xkk_1;
            kf.Pxykk_1 = kf.Pxkk_1*kf.Hk';    kf.Pykk_1 = kf.Hk*kf.Pxykk_1 + kf.Rk;
        end
        % filtering
        kf.Kk = kf.Pxykk_1*kf.Pykk_1^-1;
        kf.xk = kf.xkk_1 + kf.Kk*(yk-kf.ykk_1);
        kf.Pxk = kf.Pxkk_1 - kf.Kk*kf.Pykk_1*kf.Kk';  kf.Pxk = (kf.Pxk+kf.Pxk')/2;
    end
