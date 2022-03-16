function kf = ekf(kf, yk, TimeMeasBoth)
% Extended Kalman filter for nonlinear system.
%
% Prototype: kf = ekf(kf, yk)
% Inputs: kf - filter structure array
%         yk - measurement vector
%         TimeMeasBoth - described as follows,
%            TimeMeasBoth='T' (or nargin==1) for time updating only, 
%            TimeMeasBoth='M' for measurement updating only, 
%            TimeMeasBoth='B' (or nargin==2) for both time and 
%                             measurement updating.
% Output: kf - output filter structure array
%
% See also  ekfJcb, ukf, ckf, kfupdate.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/09/2012, 04/03/2022
    if nargin==1;
        TimeMeasBoth = 'T';
    elseif nargin==2
        TimeMeasBoth = 'B';
    end

    if TimeMeasBoth=='T' || TimeMeasBoth=='B'
        if isfield(kf, 'fx')  % nonliear state Jacobian matrix
            [kf.Phikk_1, kf.xkk_1] = ekfJcb(kf.fx, kf.xk, kf.px);
            if isempty(kf.xkk_1), kf.xkk_1 = kf.Phikk_1*kf.xk; end
        else
            kf.xkk_1 = kf.Phikk_1*kf.xk;
        end
        kf.Pxkk_1 = kf.Phikk_1*kf.Pxk*kf.Phikk_1' + kf.Gammak*kf.Qk*kf.Gammak';
        if TimeMeasBoth=='T'    % time updating only
            kf.xk = kf.xkk_1; kf.Pxk = kf.Pxkk_1;
            return;
        end
    end

    if TimeMeasBoth=='M' || TimeMeasBoth=='B'
        if TimeMeasBoth=='M'    % meas updating only
            kf.xkk_1 = kf.xk; kf.Pxkk_1 = kf.Pxk;
        end
        if isfield(kf, 'hx')  % nonliear measurement Jacobian matrix
            [kf.Hk, kf.ykk_1] = ekfJcb(kf.hx, kf.xkk_1, kf.py);
            if isempty(kf.ykk_1), kf.ykk_1 = kf.Hk*kf.xkk_1; end
        else
            kf.ykk_1 = kf.Hk*kf.xkk_1;
        end
        kf.Pxykk_1 = kf.Pxkk_1*kf.Hk';    kf.Pykk_1 = kf.Hk*kf.Pxykk_1 + kf.Rk;
        % filtering
        kf.Kk = kf.Pxykk_1*kf.Pykk_1^-1;
        kf.xk = kf.xkk_1 + kf.Kk*(yk-kf.ykk_1);
        kf.Pxk = kf.Pxkk_1 - kf.Kk*kf.Pykk_1*kf.Kk';  kf.Pxk = (kf.Pxk+kf.Pxk')/2;
    end
    
