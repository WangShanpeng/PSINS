function y = polyvaln(p, x)
% See also  polyintn, polycross, polydot, polydotmul, polyadd, polyfitn.

% Copyright(c) 2009-2018, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/10/2018, 25/01/2023
    if size(x,2)==1, x=repmat(x, 1, size(p,1));  end
    y = x;
    for n=1:size(p,1)
        y(:,n) = polyval(p(n,:), x(:,n));
    end