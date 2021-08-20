function sd = surfpk(pk)
% 3-D colored surface for covariance matrix Pk.
%
% Prototype: surfpk(pk)
% Input: pk - covariance matrix Pk
% Output: sd - sqrt(diag(Pk))
%
% See also  NA.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/01/2020
    global glv
    sd = sqrt(diag(pk)); len = length(sd);
    d1 = diag(1./sd);
    myfigure;
    subplot(2,3,[1,2, 4,5]);
    pk1 = d1*pk*d1; pk1(end+1,end+1) = pk1(end,end);
    pk1(end,1:end-1) = pk1(end-1,1:end-1); pk1(1:end-1,end) = pk1(1:end-1,end-1);
    surf(pk1); xlabel('i'); ylabel('j'); xlim([1,len+1]); ylim([1,len+1]);
    subplot(2,3,[3,6]);
    if len>=15
        s = [[1;1;1]*glv.min; [1;1;1]; [1;1]/glv.Re;1; [1;1;1]*glv.dph; [1;1;1]*glv.ug];
        sd(1:15) = sd(1:15)./s;
    end
    semilogx(sd+eps, -(1:len), '-.o'); grid on; ylim([-len-1,-1]); xlabel('sqrt(P(i,i))'); ylabel('-i');