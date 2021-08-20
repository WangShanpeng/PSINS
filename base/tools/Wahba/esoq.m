function [q, mat, k, lambda] = esoq(B, iter)
% EStimator of the Optimal Quaternion (ESOQ)
% Example: [q, mat] = esoq(randn(3))
%
% See also  tr3, det3, inv3, adj3, svd3, foam, B33M44, svdest, vortech, quest, qrsvd

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/01/2020
    if nargin<2, iter=100; end
    [M, z, s, S] = B33M44(B);
    zTz = z'*z; zzT = z*z'; zTS = z'*S;
    lambda = sqrt(trace(M*M'));  lambda0 = lambda; lambda1 = lambda;
    for k=1:iter
        T = (lambda+s)*eye(3)-S;  lambda_s = lambda-s;
        gamma = det(T);  X = adj(T)*z;
        detfM = gamma*(lambda-s - 1/(lambda+s)*zTz) - 1/(lambda+s)*zTS*X;
        W = T-1/lambda_s*zzT; aISzz = adj(W);
        tradjfM = det(W) + 1/lambda_s*z'*aISzz*z + lambda_s*trace(aISzz);
        fpp = 12*lambda^2-4*trace(B*B');
%      [L, p] = chol(lambda*eye(4)-M);  [U, D] = myudut(lambda*eye(4)-M);
%      if p>0,
%          break;
%      end
%      detfM = prod(diag(L))^2;
%      tradjfM = norm(adj(L),'fro')^2;
        dlambda = detfM/tradjfM;
%         dlambda = 2*detfM*tradjfM/(2*tradjfM^2-detfM*fpp);
        lambda = lambda - dlambda;
        if abs(dlambda/lambda)<1e-15, break; end
%         fM = lambda1*eye(4)-M;
%         lambda1 = lambda1 - det(fM)/trace(adj(fM));
%         lambdak(k,:) = [detfM, tradjfM];
    end
%     lambda0 = max(eig(M));
%     figure, semilogy(abs(lambdak-lambda0*0)); grid on
	T = (lambda+s)*eye(3)-S;
	gamma = det(T);  X = adj(T)*z;
    q = [gamma; X];    q = q/sqrt(q'*q);
    if nargout>=2, mat = q2mat(q); end
