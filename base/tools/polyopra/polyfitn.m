function p = polyfitn(t, y, odr)
% See also  polyfitRLS, polyvaln, polyintn, polycross, polydot, polydotmul, polyadd.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 25/01/2023
    if nargin==2, odr=y; y=t(:,1:end-1); t=t(:,end); end  % p = polyfitn(yt, odr)
    p = zeros(size(y,2),odr+1);
    for n=1:size(y,2)
        p(n,:) = polyfit(t, y(:,n), odr);
    end