function Pxk = fkfpbeta(Pxk, beta, nfkf, pfkf)
% Discrete-time Kalman filter. 
%
% Prototype: Pxk = fkfpbeta(Pxk, beta, nfkf, pfkf)
% Inputs: Pxk - Kalman filter state variance matrix
%         bata - federated KF info sharing factor, within (0,1)
%         nfkf - common state number
%         pfkf - Pxk enlarge type 1,2 or 3
% Output: Pxk - enlarged Kalman filter state variance matrix
%
% See also  fkfpset, fkfsub, kfupdate.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 31/01/2022
    if 1e-10<beta && beta<1   % feterated KF, Eq.(6.11.26b)
        if pfkf==1,      Pxk = Pxk * (1/beta);
        elseif pfkf==2,  Pxk(1:nfkf,1:nfkf) = Pxk(1:nfkf,1:nfkf) * (1/beta);
        elseif pfkf==3,  Pxk(1:nfkf,:) = Pxk(1:nfkf,:) * sqrt(1/beta); Pxk(:,1:nfkf) = Pxk(:,1:nfkf) * sqrt(1/beta);
        end
    end
