function fkf = fkfinit(ckf, subn, subm, beta, reset, pmethod)
% federated Kalman filter initializes from centralized KF structure.
%
% Prototype: fkf = fkfinit(ckf, subn, subm, beta, pfkf)
% Inputs: ckf - centralized KF structure;
%         subn,subm - state & measurement cell for sub-KF structures
%         beta - info sharing factors, beta_i==0 for no FKF feedback
%         reset - local system reset flag, 1=yes, 0=no
%         pmethod - local system variance enlarge method, =1,2 or 3
% Output: fkf - federated Kalman filter structure cell. NOTE: fkf{end} is master-KF
%
% See also  fkfupdate, kfinit, kfinit0.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/01/2022
    len = length(subn);
    if nargin<6,  pmethod=3;  end
    if nargin<5,  reset=1;  end;  if length(reset)==1, reset=repmat(reset,len+1,1); end
    if nargin<4,  beta=ones(len,1)/len;  end  % equal info sharing factors 1/N & beta_m = 0
    if len==length(beta),  beta(len+1)=1-sum(beta);  end  % beta_m
    if beta(len+1)<0,  error('FKF infomation sharing factor beta should be >=0 !');  end
    subn{len+1} = subn{1}; subm{len+1} = []; 
    for k=1:len+1, subn{end} = intersect(subn{end},subn{k}); end
    for k=1:len+1
        sn = subn{k}; sm = subm{k};
        skf = [];  % sub-KF structure
        skf.Phikk_1 = ckf.Phikk_1(sn,sn);  skf.Qt = ckf.Qt(sn,sn);
        skf.Hk = ckf.Hk(sm,sn);  skf.Rk = ckf.Rk(sm,sm);
        skf.Pxk = ckf.Pxk(sn,sn);
        skf.subn = sn;  skf.subm = sm; skf.ncom = length(subn{end});
        skf.fkfbeta = beta(k);  skf.fkfreset = reset(k);  skf.pmethod = pmethod; % skf.ncom for common system state number
        skf = kfinit0(skf, 1);
        fkf{k} = skf;
    end
    