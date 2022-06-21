function [fkf, testHk] = fkfupdate(ckf, fkf, yk)
% federated Kalman filter updating.
%
% Prototype: fkf = fkfupdate(ckf, fkf, zk)
% Inputs: ckf - centralized KF structure, just for FKF Phikk_1/Hk setting;
%         fkf - federated Kalman filter structure cell
%         yk - measurement array same as input to ckf
% Outputs: fkf - federated Kalman filter structure cell
%          testHk - must be 0, indicating CKF measurement consistency with FKF
%
% See also  fkfinit, kfupdate, kfinit.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/01/2022
    len = length(fkf);
    zk = cell(len,1);
    if nargin<3, for k=1:len, zk{k}=[]; end;
    else, for k=1:len, zk{k}=yk(fkf{k}.subm); end;   end
    iPg = 0;  iPkxk = 0;
    fHk = ckf.Hk*0;
    for k=1:len
        if fkf{k}.fkfbeta<1e-10, continue; end  % fkfbeta<=0 for no info fusion fkf{k}
        sm = fkf{k}.subm; sn = fkf{k}.subn;  ncom = fkf{k}.ncom;
        fkf{k}.Phikk_1 = ckf.Phikk_1(sn,sn);  fkf{k}.Qk = ckf.Qk(sn,sn);  % NOTE: ckf.Gammak=1
        fkf{k}.Hk = ckf.Hk(sm,sn);  fkf{k}.Rk = ckf.Rk(sm,sm);
        fHk(sm,sn) = ckf.Hk(sm,sn);
        
        if fkf{k}.fkfreset==1 % fkfreset==0 for no reset fkf{k}
            if k<len  % reset X,P except master-KF 
                fkf{k}.xk(1:ncom) = fkf{end}.xk;  % NOTE: common system states must be the first 1:nfkf elements 
                sP = chol(fkf{k}.Pxk(1:ncom,1:ncom)); sPfkf = chol(fkf{end}.Pxk);  % P = sP' * sP;
                A = sP^-1 * sPfkf;    %    A'*sP'*sP*A = sPfkf'*sPfkf => sP*A = sPfkf => A = sP^-1 * sPfkf
                fkf{k}.Pxk = [fkf{end}.Pxk, A'*fkf{k}.Pxk(1:ncom,ncom+1:end); fkf{k}.Pxk(ncom+1:end,1:ncom)*A, fkf{k}.Pxk(ncom+1:end,ncom+1:end)];
            end

            fkf{k}.Pxk = PQbeta(fkf{k}.Pxk, fkf{k}.fkfbeta, ncom, fkf{k}.pmethod);  % enlarge P,Q
            fkf{k}.Qk  = PQbeta(fkf{k}.Qk,  fkf{k}.fkfbeta, ncom, fkf{k}.pmethod);
        end
        
        if isempty(zk{k}), fkf{k} = kfupdate(fkf{k});  % KF update
        else  fkf{k} = kfupdate(fkf{k}, zk{k});  end
        
        iPk = invbc(fkf{k}.Pxk(1:ncom,1:ncom)); iPg = iPg + iPk;  % FKF info fusion
        iPkxk = iPkxk + iPk*fkf{k}.xk(1:ncom);
    end
    fkf{end}.Pxk = invbc(iPg);  fkf{end}.xk = fkf{end}.Pxk * iPkxk;  % record Pg, Xg -> Pxk, xk
    testHk = norm(fHk-ckf.Hk,inf);
    
function Pxk = PQbeta(Pxk, beta, ncom, pmethod)   % enlarge P,Q, Eq.(6.11.26b)
	if pmethod==1,
        Pxk = Pxk * (1/beta);
	elseif pmethod==2,
        Pxk(1:ncom,1:ncom) = Pxk(1:ncom,1:ncom) * (1/beta);
	elseif pmethod==3,
        sb = sqrt(1/beta);
        Pxk(1:ncom,:) = Pxk(1:ncom,:) * sb; Pxk(:,1:ncom) = Pxk(:,1:ncom) * sb;
    end

