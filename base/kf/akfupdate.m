function kf = akfupdate(kf, yk, TimeMeasBoth, kftype)
% Adaptive Kalman filter. 
%
% Prototype: kf = akfupdate(kf, yk, TimeMeasBoth, kftype)
% Inputs: kf - Kalman filter structure array
%         yk - measurement vector
%         TimeMeasBoth - described as follows,
%            TimeMeasBoth='T' (or nargin==1) for time updating only, 
%            TimeMeasBoth='M' for measurement updating only, 
%            TimeMeasBoth='B' (or nargin==2) for both time and 
%                             measurement updating.
%        kftype - adaptive KF type
% Output: kf - Kalman filter structure array after time/meas updating
%
% The MCKF/RSTKF/SSMKF source code is provided by Lianzhao Wang.
%
% See also  kfinit, kfupdate.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/03/2022
    if nargin==1;
        TimeMeasBoth = 'T';
    elseif nargin==2
        TimeMeasBoth = 'B';
    end
    
    if TimeMeasBoth=='T'            % Time Updating
        kf.xk = kf.Phikk_1*kf.xk;    
        kf.Pxk = kf.Phikk_1*kf.Pxk*kf.Phikk_1' + kf.Gammak*kf.Qk*kf.Gammak';
    else
        if TimeMeasBoth=='M'        % Meas Updating
            kf.xkk_1 = kf.xk;
            kf.Pxkk_1 = kf.Pxk;
        elseif TimeMeasBoth=='B'    % Time & Meas Updating
            kf.xkk_1 = kf.Phikk_1*kf.xk;
            kf.Pxkk_1 = kf.Phikk_1*kf.Pxk*kf.Phikk_1' + kf.Gammak*kf.Qk*kf.Gammak';
        else
            error('TimeMeasBoth input error!');
        end
        if ischar(kftype)
            switch(kftype)
                case 'AKF',  kftype=1;  case 'MCKF',  kftype=2;  case 'RSTKF', kftype=3;  case 'SSMKF',  kftype=4;
            end
        end
        kf.lambda = 1;
        Rk = kf.Rk;
        xk_1 = kf.xk+1;
        if ~isfield(kf,''), kf.measIter=10; end
        for iter = 1:kf.measIter         
            kf.Pxykk_1 = kf.Pxkk_1*kf.Hk';
            kf.Py0 = kf.Hk*kf.Pxykk_1;
            kf.ykk_1 = kf.Hk*kf.xkk_1;
            kf.rk = yk-kf.ykk_1;
            kf.Pykk_1 = kf.Py0 + Rk;
            kf.Kk = kf.Pxykk_1*invbc(kf.Pykk_1);
            kf.Pxk = kf.Pxkk_1 - kf.Kk*kf.Pykk_1*kf.Kk';  kf.Pxk = (kf.Pxk+kf.Pxk')*(1/2);
            kf.xk = kf.xkk_1 + kf.Kk*kf.rk;
            if (norm(kf.xk-xk_1)/norm(kf.xk))<1e-12, break; else, xk_1 = kf.xk; end
            
            Bk = (yk-kf.Hk*kf.xk)*(yk-kf.Hk*kf.xk)' + kf.Hk*kf.Pxk*kf.Hk';
            switch(kftype)
                case 1,  % AKF³£¹æ×ÔÊÊÓ¦¿¨¶ûÂüÂË²¨ by Yan Gongmin
                    Rk = (kf.rk*kf.rk' + kf.Rk)/2;
                    continue;
                case 2,  % MCKF×î´óÏà¹ØìØ¿¨¶ûÂüÂË²¨
                    tbr = Bk*kf.Rk^(-1);
                    kf.lambda = -2*(-0.5*exp((1-trace(tbr))/2/10/10));
                case 3,  % RSTKFÂ³°ôÑ§Éút¿¨¶ûÂüÂË²¨
                    v = 10;
                    tbr = Bk*kf.Rk^(-1);
                    kf.lambda = -2*(-0.5*(v + 1)/(v + trace(tbr)));
                case 4,  % SSMKFÍ³¼ÆÏàËÆ¶ÈÁ¿¿¨¶ûÂüÂË²¨
                    w = 5;
                    tbr = Bk*kf.Rk^(-1);
                    kf.lambda = -2*(-0.5*sqrt((w + 1)/(w + trace(tbr))));
            end
            if kf.lambda<1e-6, kf.lambda = 1e-6;  end
            Rk = kf.Rk/kf.lambda;
        end
    end
    kf.lambda = iter;
