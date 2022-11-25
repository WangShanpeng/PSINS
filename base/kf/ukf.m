function kf = ukf(kf, yk, TimeMeasBoth)
% Unscented Kalman filter for nonlinear system.
%
% Prototype: kf = ukf(kf, yk, TimeMeasBoth)
% Inputs: kf - filter structure array
%         yk - measurement vector
%         TimeMeasBoth - described as follows,
%            TimeMeasBoth='T' (or nargin==1) for time updating only, 
%            TimeMeasBoth='M' for measurement updating only, 
%            TimeMeasBoth='B' (or nargin==2) for both time and 
%                             measurement updating.
% Output: kf - filter structure array after time/meas updating
%
% See also  ukfUT, ckf, ssukf, ekf, kfupdate.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/09/2012
    if nargin==1;
        TimeMeasBoth = 'T';
    elseif nargin==2
        TimeMeasBoth = 'B';
    end

    if ~isfield(kf, 'alpha')
        kf.alpha = 1e-3; kf.beta = 2; kf.kappa = 0;
    end
    
    if TimeMeasBoth=='T' || TimeMeasBoth=='B'
        if isfield(kf, 'fx')  % nonliear state propagation
            [kf.xkk_1, kf.Pxkk_1] = ukfUT(kf.xk, kf.Pxk, kf.fx, kf.px, kf.alpha, kf.beta, kf.kappa);
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
            [kf.ykk_1, kf.Pykk_1, kf.Pxykk_1] = ukfUT(kf.xkk_1, kf.Pxkk_1, kf.hx, kf.py, kf.alpha, kf.beta, kf.kappa);
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
