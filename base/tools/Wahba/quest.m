function [q, mat] = quest(A)
% QUaternion ESTimator (QUEST)
% Example: [q, mat] = quest(randn(3));
%
% See also  tr3, det3, inv3, adj3, svd3, foam, B33M44, svdest, vortech, maxeig, esoq, qrsvd

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/01/2020
    M = A;
	if size(A,1)==3, M = B33M44(A); end
    [V,D] = eig(M); D = diag(D);
    [m,I] = max(D);
    q = V(:,I);
    if nargout==2, mat = q2mat(q); end

%     Hx1=M(1,1);    Hx2=M(1,2);    Hx3=M(1,3);
%     Hy1=M(2,1);    Hy2=M(2,2);    Hy3=M(2,3);
%     Hz1=M(3,1);    Hz2=M(3,2);    Hz3=M(3,3);
%     W=[Hx1+Hy2+Hz3,-Hy3+Hz2,-Hz1+Hx3,-Hx2+Hy1;
%         -Hy3+Hz2, Hx1-Hy2-Hz3,Hx2+Hy1,Hx3+Hz1;
%         -Hz1+Hx3,Hx2+Hy1,Hy2-Hx1-Hz3,Hy3+Hz2;
%         -Hx2+Hy1,Hx3+Hz1,Hy3+Hz2,Hz3-Hy2-Hx1];
%     c=det(W);
%     b=8*Hx3*Hy2*Hz1 - 8*Hx2*Hy3*Hz1 - 8*Hx3*Hy1*Hz2 + 8*Hx1*Hy3*Hz2 + 8*Hx2*Hy1*Hz3 - 8*Hx1*Hy2*Hz3;
%     a=-2*Hx1*Hx1 - 2*Hx2*Hx2 - 2*Hx3*Hx3 - 2*Hy1*Hy1 - 2*Hy2*Hy2 - 2*Hy3*Hy3 - 2*Hz1*Hz1 - 2*Hz2*Hz2 - 2*Hz3*Hz3;
%     T0 = 2*a^3 + 27*b^2 - 72*a*c;
%     T1 = (T0 + sqrt(-4*(a^2 + 12*c)^3 + T0^2))^(1/3);
%     T2 = sqrt(-4*a + 2^(4/3)*(a^2 + 12*c)/T1 + 2^(2/3)*T1);
%     m1 = 0.20412414523193150818310700622549*(T2 + sqrt(-T2^2 - 12*a - 12 *2.4494897427831780981972840747059*b/T2));
%     m1 = real(m1);
%     for k=1:20
%         fM = m1*eye(4)-M;
%         m1 = m1 - det(fM)/trace(adj(fM));
%         mk(k) = m1;
%     end
%     figure, plot(mk-m); grid on
%     