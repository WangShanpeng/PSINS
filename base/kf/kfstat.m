function kfs = kfstat(kfs, kf, flag)
% Kalman filter error distribution analysis and statistic.
% Ref. 'Yan G. Error Distribution Method and Analysis of Observability Degree 
%      Based on the Covariances in Kalman Filter, CCC2018'.
%
% See also  alignvn_kfs, kfupdate.

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
        IKH = kf.I-kf.Kk*kf.Hk;  Akk_1 = IKH*kf.Phikk_1;  Bkk_1 = IKH*kf.Gammak;
        if nargin==2, flag='B'; end
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
            for s=1:kf.m
                kfs.Rsk{s} = Akk_1*kfs.Rsk{s}*Akk_1';
                kfs.Pk1 = kfs.Pk1 + kfs.Rsk{s};
            end
            kfs.Pk = kf.Phikk_1*kfs.Pk*kf.Phikk_1'+kf.Gammak*kf.Qk*kf.Gammak';
%             b=[kfs.Pk-kfs.Pk1]; max(max(abs(b)))
        elseif flag=='M'
            kfs.Ak0 = Akk_1*kfs.Ak0;
            kfs.Pk1 = kfs.Ak0*kfs.P0*kfs.Ak0';
            for j=1:kf.l
                kfs.Qjk{j} = Akk_1*kfs.Qjk{j}*Akk_1';
                kfs.Pk1 = kfs.Pk1 + kfs.Qjk{j};
            end
            for s=1:kf.m
                kfs.Rsk{s} = Akk_1*kfs.Rsk{s}*Akk_1' + kf.Rk(s,s)*kf.Kk(:,s)*kf.Kk(:,s)';
                kfs.Pk1 = kfs.Pk1 + kfs.Rsk{s};
            end
            kfs.Pk = Bkk_1*kfs.Pk*Bkk_1'+kf.Kk*kf.Rk*kf.Kk';
        end         
    elseif nargin==2 && ~isempty(kfs)   %  plot,   kfs = kfstat(kfs, kf)
        n = length(kfs.P0); m = length(kfs.Rsk);
        p = zeros(n); q = zeros(n,kf.l); r = zeros(n,m);
        kfs.Pk0 = kfs.Ak0*kfs.P0*kfs.Ak0';
        for ll=1:n
%             Pkll = 1;
            Pkll = kfs.Pk(ll,ll)+eps*eps;
            for jj=1:n
                p(ll,jj) = kfs.Ak0(ll,jj)*kfs.P0(jj,jj)*kfs.Ak0(ll,jj)/Pkll;
            end
            for jj=1:kf.l
                q(ll,jj) = kfs.Qjk{jj}(ll,ll)/Pkll;
            end
            for ss=1:m
                r(ll,ss) = kfs.Rsk{ss}(ll,ll)/Pkll;
            end
        end
        kfs.p = p; kfs.q = q; kfs.r = r;
        kfs.err = [sum([p, q, r],2) - diag(kfs.Pk)];
%         for ll=1:2*n+m, kfs.pqr(:,ll) = kfs.pqr(:,ll)/max(abs(kfs.pqr(:,ll))); end
%         for ll=1:n, kfs.pqr(ll,:) = kfs.pqr(ll,:)/max(abs(kfs.pqr(ll,:))); end
        myfigure,% mesh(repmat((1:n)',1,2*n+m),repmat(1:2*n+m,n,1),kfs.pqr);
        if Pkll==1
            kfs.pqr = sqrt([p, q, r]);
            subplot(321), bar(kfs.pqr(1:3,:)'/glv.min); grid on
            subplot(322), bar(kfs.pqr(4:6,:)'); grid on
            subplot(323), bar([kfs.pqr(7:8,:)'*glv.Re,kfs.pqr(9,:)']); grid on
            subplot(324), bar(kfs.pqr(10:12,:)'/glv.dph); grid on
            subplot(325), bar(kfs.pqr(13:15,:)'/glv.ug); grid on
            subplot(326), bar(kfs.pqr(16:18,:)'); grid on
        else
            kfs.pqr = [p, q, r]*100;
            subplot(221), bar(kfs.pqr(1:3,:)'); title('( a )'); xygo('j', 'Percentage'); legend('\phi_E', '\phi_N', '\phi_U')
            subplot(222), bar(kfs.pqr(4:6,:)'); title('( b )'); xygo('j', 'Percentage'); legend('\deltav^n_E', '\deltav^n_N', '\deltav^n_U')
            subplot(223), bar(kfs.pqr(7:9,:)'); title('( c )'); xygo('j', 'Percentage'); legend('\epsilon^b_x', '\epsilon^b_y', '\epsilon^b_z')
            subplot(224), bar(kfs.pqr(10:12,:)'); title('( d )'); xygo('j', 'Percentage'); legend('\nabla^b_x', '\nabla^b_y', '\nabla^b_z')
        end
    end
        