function fkf = fkfinit(ckf, subn, subm, beta, pfkf)
% federated Kalman filter initializes from centralized KF structure.
%
% Prototype: fkf = fkfinit(ckf, subn, subm, beta, pfkf)
% Inputs: ckf - centralized KF structure;
%         subn,subm - state & measurement cell for sub-KF structures
%         beta - info sharing factors
%         pfkf - variance calculating type
% Output: fkf - federated Kalman filter structure cell. NOTE: fkf{end} is master-KF
%
% See also  fkfupdate, kfinit, kfinit0.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/01/2022
    if nargin<5,  pfkf=3;  end
    if nargin<4,  beta=ones(length(subn),1)/length(subn);  end  % equal info sharing factors 1/N & beta_m = 0
    len = length(beta);
    beta(len+1) = 1-sum(beta);  % beta_m
    if beta(len+1)<0,  error('FKF beta should be >=0 !');  end
    subn{len+1} = subn{1}; subm{len+1} = []; 
    for k=1:len+1, subn{end} = intersect(subn{end},subn{k}); end
    for k=1:len+1
        sn = subn{k}; sm = subm{k};
        skf = [];  % sub-KF structure
        skf.Phikk_1 = ckf.Phikk_1(sn,sn);  skf.Qt = ckf.Qt(sn,sn);
        skf.Hk = ckf.Hk(sm,sn);  skf.Rk = ckf.Rk(sm,sm);
        skf.Pxk = ckf.Pxk(sn,sn);
        skf.subn = sn;  skf.subm = sm;  skf.fbeta = beta(k);  skf.nfkf = length(subn{end}); skf.pfkf = pfkf;  % skf.nfkf for common state number
        skf = kfinit0(skf, 1);
        fkf{k} = skf;
    end
    