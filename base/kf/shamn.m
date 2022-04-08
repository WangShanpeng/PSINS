function [Rk, beta, maxflag] = shamn(Rk, r2, Rmin, Rmax, beta, b, s)
% Sage-Husa adaptive measurement noise variance 'Rk'. 
%
% Prototype: [Rk, beta] = shamn(Rk, r2, Rmin, Rmax, beta, b, s)
% Inputs: Rk - measurement noise variance
%         r2 - =rk^2-Py0;
%         Rmin,Rmax - min & max variance bound.
%         beta,b - forgettiing factors
%         s - if b==0, s is enlarge factor
% Output: Rk, beta - same as above
%         maxflag - if r2(k)>Rmax(k) then set the max flag
%
% See also  akfupdate.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/03/2022
    m = length(r2);  maxflag = zeros(m,1);
    beta = beta./(beta+b);
    for k=1:m
        if b(k)==0,
            r2(k) = r2(k) / s(k)^2;
            if r2(k)<Rmin(k), Rk(k,k)=Rmin(k); else, Rk(k,k)=r2(k);  end
        else
            if r2(k)<Rmin(k), r2(k)=Rmin(k);
            elseif r2(k)>Rmax(k), r2(k)=Rmax(k); beta(k)=1; maxflag(k)=1; end
            Rk(k,k) = (1-beta(k))*Rk(k,k) + beta(k)*r2(k);
        end
    end
