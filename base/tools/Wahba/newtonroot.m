function [x, k] = newtonroot(p, x0, iter)
% Find polynomial root using Newton's method, initialized by coarse root x0.
% See also  tr3, det3, inv3, adj3, svd3, foam

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/01/2020
    if nargin<3, iter = 100; end
    if nargin<2, x0 = 0; end
    dp = polyder(p);
    x = x0;
    for k=1:iter
%         xy(k,:) = [x, polyval(p,x), polyval(dp,x)];
        dx = polyval(p,x)/polyval(dp,x);
        x = x - dx;
        if abs(dx)<1e-50; break; end
    end
%     figure, plot(xy(:,1), xy(:,2:3),'-x'); grid on
