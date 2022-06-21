function [y, Pyy, Pxy, X, Y] = ckfCT(x, Pxx, hfx, tpara)
% Spherical-Radial Cubature transformation.
%
% Prototype: [y, Pyy, Pxy, X, Y] = ckfCT(x, Pxx, hfx, tpara)
% Inputs: x, Pxx - state vector and its variance matrix
%         hfx - a handle for nonlinear state equation
%         tpara - some time-variant parameter pass to hfx
% Outputs: y, Pyy - state vector and its variance matrix after UT
%          Pxy - covariance matrix between x & y
%          X, Y - Sigma-point vectors before & after UT
%
% See also  ckf, ukfUT, SSUT.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/03/2022
    n = length(x);
    sPxx = sqrt(n)*chol(Pxx)';    % Choleskey decomposition
    xn = repmat(x,1,n); 
    X = [xn+sPxx, xn-sPxx];
    y = feval(hfx, X(:,1), tpara);
    Y = repmat(y,1,2*n);
    Pyy = Y(:,1)*Y(:,1)'; Pxy = X(:,1)*Y(:,1)';
    for k=2:1:2*n
        Y(:,k) = feval(hfx, X(:,k), tpara);
        y = y + Y(:,k);
        Pyy = Pyy + Y(:,k)*Y(:,k)';
        Pxy = Pxy + X(:,k)*Y(:,k)';
    end
    y = 1/(2*n)*y;  % y mean
    Pyy = 1/(2*n)*Pyy - y*y';  Pxy = 1/(2*n)*Pxy - x*y';
 
%     Y(:,1) = feval(hfx, X(:,1), tpara); m=length(Y); y = Y(:,1);
%     Y = repmat(Y,1,2*n);
%     Pyy = zeros(m); Pxy = zeros(n,m);
%     for k=2:1:2*n     % Sigma points nolinear propagation
%         Y(:,k) = feval(hfx, X(:,k), tpara);
%         y = y + Y(:,k);
%     end
%     y = 1/(2*n)*y;
%     for k=1:1:2*n
%         yerr = Y(:,k)-y;
%         Pyy = Pyy + (yerr*yerr');  % variance
%         xerr = X(:,k)-x;
%         Pxy = Pxy + xerr*yerr';  % covariance
%     end
%     Pyy = 1/(2*n)*Pyy; Pxy = 1/(2*n)*Pxy;
