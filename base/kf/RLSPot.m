function kf = RLSPot(kf, zk)
% Recursive Least Square filter using Potter SR-KF.
%
% Prototype: kf = RLS(kf, zk)
% Inputs: kf - filter structure array, kf.Hk should be row-vector
%         zk - measurement vector
% Output: kf - filter structure array after filtering
%
% See also  RLS, RLSUD, kfupdate.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/02/2023
    if ~isfield(kf, 'Rk') 
        kf.Rk = 1;
    end
    HD = kf.Hk*kf.Delta;
    kf.Pzk = HD*HD' + kf.Rk;
    kf.Kk = kf.Delta*HD'*kf.Pzk^-1;
    kf.xk = kf.xk + kf.Kk*(zk-kf.Hk*kf.xk);
%     [q, r] = qr(single([HD';sqrt(kf.Rk)])); rho = r(1);
    rho = sqrt(kf.Pzk);  
    gamma = rho*(rho-sqrt(kf.Rk));
    kf.Delta = kf.Delta*(eye(length(kf.xk))-1/gamma*HD'*HD);
    kf.Pxk = kf.Delta*kf.Delta';  % for debug
