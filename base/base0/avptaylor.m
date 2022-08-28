function [Ct, Vt, Pt, iter] = avptaylor(C0, V0, P0, Wt, Ft, T, tol)
% Solution for AVP differential equation in inertial frame.
% Ref. '一种无误差的捷联惯导数值更新新算法'
% 
% Prototype: [Ct, Vt, Pt, iter] = avptaylor(C0, V0, P0, Wt, Ft, T, tol)
% Inputs: C0, V0, P0 - Init DCM,vel,pos 
%         Wt, Ft - 3xn angluar rate / specific force  coefficients of the 
%                 polynomial in descending powers
%         T - one step forward for time 0 to T
%         tol - error tolerance
% Outputs: Ct, Vt, Pt - output DCM,vel,pos at time T
%          iter - iteration count
% 
% See also  dcmtaylor, qtaylor, qpicard, wm2wtcoef.

% Copyright(c) 2009-2017, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 04/02/2017
    if ~exist('tol','var'), tol=1e-20; end
    sz2 = size(Wt,2);
    der = 1;
    for k=sz2:-1:1  % [AWt(n), ..., AWt(1), AWt(0)]
        AWt(:,:,k) = askew(Wt(:,k)*der); dFt(:,k) = Ft(:,k)*der; der = der*(sz2-k+1);  % polyder
    end
    yh = zeros(1,sz2);  yh(end) = 1; tsn1 = 1;
    C0i = zeros(3,3,sz2); C0i(:,:,end) = C0; % [C0(k+0), C0(k+1), ..., C0(k+n)]
    Ct = C0;
    Vt = V0;
    Pt = P0+T*V0;
    m = 250;
    for iter=1:m
        k1 = max(sz2-iter+1,1);
        C0i1 = zeros(3); V0i = zeros(3,1);
        for k=k1:sz2
            C0i1 = C0i1 + yh(k)*C0i(:,:,k)*AWt(:,:,k);
            V0i  = V0i  + yh(k)*C0i(:,:,k)*dFt(:,k);
        end
        tsn1 = tsn1*T/iter;   % ts^iter/factorial(iter)
        Ct = Ct + tsn1*C0i1;
        C0i(:,:,1:end-1) = C0i(:,:,2:end); C0i(:,:,end) = C0i1;
        Vt = Vt + tsn1*V0i;
        Pt = Pt + tsn1*T/(iter+1)*V0i;
        if iter>sz2 && tsn1*max(max(abs(C0i1)))<tol && norm(tsn1*V0i)<tol*tol, break; end
        yh = [yh(2:end),0] + yh;  % Yanghui coefficients
    end
    Ct = mnormlz(Ct);

