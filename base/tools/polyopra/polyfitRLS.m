function [p, p0k] = polyfitRLS(t, y, odr)
% See also  polyfitn, polyvaln.
%
% Example
%   y=randn(1000,1); y([100,end-10])=100;  t=(1:length(y))';  p1=polyfitn(t, y, 3); p2=polyfitRLS(t, y, 3); [p1; p2]
%   myfig, plot(t, [y, polyvaln([p1;p2],t)]);

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 25/01/2023
    if nargin==2, odr=y; t=y(:,end); y=y(:,1:end-1); end
    [len, m] = size(y);
    p = zeros(m,odr+1);  p0k = y;
    Pk = 1e4*eye(odr+1);  Rk = 1;  odr=odr:-1:0;
    if len>1000, timebar(1, len); end
    for k=1:len
        Hk = t(k).^odr;
        Pxz = Pk*Hk';  Pz = Hk*Pxz+Rk; Kk = Pxz/Pz;
        Pk = Pk - Kk*Pxz';  Pk = (Pk+Pk')/2;
        for n=1:m
            x = p(n,:)';
            x = x + Kk*(y(k,n)-Hk*x);
            p(n,:) = x';
        end
        p0k(k,:) = p(:,end)';
        if len>1000, timebar(); end
    end