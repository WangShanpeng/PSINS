function [M, z, s, S, r, cnd] = B33M44(B)
% Used by QUEST
% example:
%    M = B33M44(randn(3));
%    M = B33M44(a2mat(randn(3,1)));
% See also  tr3, det3, inv3, adj3, svd3, foam, quest

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/01/2020

    s = trace(B); z = [B(3,2)-B(2,3); B(1,3)-B(3,1); B(2,1)-B(1,2)]; S = B+B';
    M = [s, z'; z, S-s*eye(3)];
    if nargout>=5
        detB = det(B);
        adjBp = adj3(B');  adjBp2 = tr3(adjBp*adjBp');
        BBp = B*B';  B2 = tr3(BBp);
        b = -2*B2; c = -8*detB; d = B2^2-4*adjBp2;
        D = - 8*b;
        E = - 8*c;
        F = 16*b^2 - 64*d;
        A = D^2 - 3*F;
        Bs = D*F - 9*E^2;
        fail = 0;
        if A<0, sA=0; fail=1; else
            sA = sqrt(A); end
        y = (3*Bs-2*A*D)/(2*A*sA);
        if y>1, y=1; fail=1; 
        elseif y<-1, y=-1; fail=1; end
        theta = acos(y);
        ct3 = cos(theta/3); st3 = sin(theta/3);
        y1 = (D-2*sA* ct3             )/1;
        y2 = (D+  sA*(ct3+sqrt(3)*st3))/1;
        y3 = (D+  sA*(ct3-sqrt(3)*st3))/1;
        if y1<0, sy1=0; fail=1; else
            sy1 = sqrt(y1); end
        if y2<0, sy2=0; fail=1; else
            sy2 = sqrt(y2); end
        if y3<0, sy3=0; fail=1; else
            sy3 = sqrt(y3); end
        r = [(sy1+sy2+sy3)/(4*sqrt(3)), sqrt(B2+2*sqrt(adjBp2)), max(eig(M)), fail];
        cnd = [sqrt(B2*adjBp2)/abs(detB), max([sy1,sy2,sy3])/min([sy1,sy2,sy3]), cond(B)];
%         r0 = max(eig(M)); [r, r-r0, (r-r0)/r0], [[sy1,sy2,sy3]'/(4*sqrt(3)),svd(B)]
        %% FLAE should complex
%         T0 = 2*b^3 + 27*c^2 - 72*b*d;
%         T1 = -4*(b^2 + 12*d)^3 + T0^2;
%         if T1<0, T1 = 0; end
%         T1 = T0 + sqrt(T1);
% %         T1 = T0 + sqrt(-4*(b^2 + 12*d)^3 + T0^2);
%         if T1<0, T1 = -(-T1)^(1/3);
%         else, T1 = T1^(1/3); end
% %         T1 = (T0 + sqrt(-4*(b^2 + 12*d)^3 + T0^2))^(1/3);
%         T2 = -4*b + 2^(4/3)*(b^2 + 12*d)/T1 + 2^(2/3)*T1;
%         if T2<0, T2 = -sqrt(-T2);
%         else, T2 = sqrt(T); end
% %         T2 = sqrt(-4*b + 2^(4/3)*(b^2 + 12*d)/T1 + 2^(2/3)*T1);
%         x = -T2^2 - 12*b - 12 *sqrt(6)*c/T2;
%         if x<0, x = -sqrt(-x); 
%         else, x = sqrt(x); end
%         lambda2 = sqrt(1/24)*(T2 + x);
% %         lambda2 = sqrt(1/24)*(T2 + sqrt(-T2^2 - 12*b - 12 *sqrt(6)*c/T2));
%         [r, lambda2, max(eig(M))]
    end
%     M = [ B(1,1)+B(2,2)+B(3,3),-B(2,3)+B(3,2),       -B(3,1)+B(1,3),       -B(1,2)+B(2,1);
%          -B(2,3)+B(3,2),        B(1,1)-B(2,2)-B(3,3), B(1,2)+B(2,1),        B(1,3)+B(3,1);
%          -B(3,1)+B(1,3),        B(1,2)+B(2,1),        B(2,2)-B(1,1)-B(3,3), B(2,3)+B(3,2);
%          -B(1,2)+B(2,1),        B(1,3)+B(3,1),        B(2,3)+B(3,2),        B(3,3)-B(2,2)-B(1,1) ];

%     ee = eig(M);
%     x = min(ee)-0.2:0.1:max(ee)+0.2;
%     y = x;
%     for k=1:length(x)
%         y(k) = det(x(k)*eye(4)-M);
%     end
%     plot(x, y, '-', ee, ee*0, 'xr'); grid on; hold on; title(sprintf('%f',det(B)));
    