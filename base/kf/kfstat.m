function kfs = kfstat(kfs, kf, flag)
% Kalman filter error distribution analysis and statistic.
% Ref. 'Yan G. Error Distribution Method and Analysis of Observability Degree 
%      Based on the Covariances in Kalman Filter, CCC2018'.
%
% See also  alignvn_kfs, tbinseval, kfupdate, sinsgps.

% Copyright(c) 2009-2018, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 11/06/2018
global glv
    if isempty(kfs),  % initialize,   kfs = kfstat([], kf)
        kfs.Ak0 = eye(kf.n);
        kfs.P0 = kf.Pxk;  kfs.Pk = kf.Pxk; kfs.Pk1 = kfs.Pk;
        for k=1:kf.l, kfs.Qjk{k} = zeros(kf.n); end
        for k=1:kf.m, kfs.Rsk{k} = zeros(kf.n); end
    elseif nargin>=3,  % update,   kfs = kfstat(kfs, kf, 'B/T/M')
        if nargin==2, flag='B'; end
        Kk = kf.Kk;
        if flag=='T', kf.Kk=kf.Kk*0; end
        IKH = kf.I-kf.Kk*kf.Hk;  Akk_1 = IKH*kf.Phikk_1;  Bkk_1 = IKH*kf.Gammak;
        if flag=='B'
            kfs.Ak0 = Akk_1*kfs.Ak0;
            kfs.Pk1 = kfs.Ak0*kfs.P0*kfs.Ak0';
            for j=1:kf.l
                kfs.Qjk{j} = Akk_1*kfs.Qjk{j}*Akk_1' + kf.Qk(j,j)*Bkk_1(:,j)*Bkk_1(:,j)';
                kfs.Pk1 = kfs.Pk1 + kfs.Qjk{j};
            end
            for s=1:kf.m
                kfs.Rsk{s} = Akk_1*kfs.Rsk{s}*Akk_1' + kf.Rk(s,s)*kf.Kk(:,s)*kf.Kk(:,s)';
                kfs.Pk1 = kfs.Pk1 + kfs.Rsk{s};
            end
            kfs.Pk = IKH*(kf.Phikk_1*kfs.Pk*kf.Phikk_1'+kf.Gammak*kf.Qk*kf.Gammak')*IKH'+kf.Kk*kf.Rk*kf.Kk';
        elseif flag=='T'
            kfs.Ak0 = Akk_1*kfs.Ak0;
            kfs.Pk1 = kfs.Ak0*kfs.P0*kfs.Ak0';
            for j=1:kf.l
                kfs.Qjk{j} = Akk_1*kfs.Qjk{j}*Akk_1' + kf.Qk(j,j)*Bkk_1(:,j)*Bkk_1(:,j)';
                kfs.Pk1 = kfs.Pk1 + kfs.Qjk{j};
            end
            kfs.Pk = kf.Phikk_1*kfs.Pk*kf.Phikk_1'+kf.Gammak*kf.Qk*kf.Gammak';
%             b=[kfs.Pk-kfs.Pk1]; max(max(abs(b)))
        elseif flag=='M'
            kfs.Ak0 = Akk_1*kfs.Ak0;
            kfs.Pk1 = kfs.Ak0*kfs.P0*kfs.Ak0';
            for s=1:kf.m
                kfs.Rsk{s} = Akk_1*kfs.Rsk{s}*Akk_1' + kf.Rk(s,s)*kf.Kk(:,s)*kf.Kk(:,s)';
                kfs.Pk1 = kfs.Pk1 + kfs.Rsk{s};
            end
            kfs.Pk = IKH*kfs.Pk*IKH'+kf.Kk*kf.Rk*kf.Kk';
        end
        kf.Kk = Kk;
    else   %  plot,   kfs = kfstat(kfs)
        n = length(kfs.P0); kfl = length(kfs.Qjk); m = length(kfs.Rsk);
        p = zeros(n); q = zeros(n,kfl); r = zeros(n,m);
        kfs.Pk0 = kfs.Ak0*kfs.P0*kfs.Ak0';
        for ll=1:n
            Pkll = kfs.Pk(ll,ll)+eps*eps;
            for jj=1:n
                p(ll,jj) = kfs.Ak0(ll,jj)*kfs.P0(jj,jj)*kfs.Ak0(ll,jj)/Pkll;
            end
            
            for jj=1:kfl
                q(ll,jj) = kfs.Qjk{jj}(ll,ll)/Pkll;
            end
            for ss=1:m
                r(ll,ss) = kfs.Rsk{ss}(ll,ll)/Pkll;
            end
        end
        kfs.p = p; kfs.q = q; kfs.r = r;  % p,q,r in Percentage
        s = sum([p, q, r],2);
        for k=1:n, kfs.p(k,:)=kfs.p(k,:)/s(k); kfs.q(k,:)=kfs.q(k,:)/s(k); kfs.r(k,:)=kfs.r(k,:)/s(k); end  % normalize
        kfs.err = sqrt(s) - 1;
    end
        