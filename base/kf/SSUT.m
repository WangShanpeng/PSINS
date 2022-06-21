function [y, Pyy, Pxy, X, Y] = SSUT(x, Pxx, hfx, tpara, w0)
% Spherical Simplex Unscented Transformation.
%
% Prototype: [y, Pyy, Pxy, X, Y] = SSUT(x, Pxx, hfx, tpara, w0)
% Inputs: x, Pxx - state vector and its variance matrix
%         hfx - a handle for nonlinear state equation
%         tpara - some time-variant parameter pass to hfx
%         w0 - weight0
% Outputs: y, Pyy - state vector and its variance matrix after UT
%          Pxy - covariance matrix between x & y
%          X, Y - Sigma-point vectors before & after UT
%
% See also  ssukf, ukfUT, ckfCT.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/06/2022
global SSUT_s  SSUT_w
    if isempty(SSUT_w), SSUT_w(1)=0; end
    if nargin<5, w0=0; end
    n = length(x);
    if n~=size(SSUT_s,1) || SSUT_w(1)~=w0
        w1 = (1-w0)/(n+1);
        s = [0, -1/sqrt(2), 1/sqrt(2)];
        for j=2:n
            s1 = [s(:,1); 0];
            for i=1:j
                s1(:,i+1) = [s(:,i+1); -1/sqrt(j*(j+1))];
            end
            s1(:,j+2) = [zeros(j-1,1);j/sqrt(j*(j+1))];
            s = s1;
        end
        SSUT_s = s/sqrt(w1);  SSUT_w = [w0, repmat(w1,1,n+1)];
    end
    %%
    X = repmat(x,1,n+2) + chol(Pxx)'*SSUT_s;
    y = SSUT_w(1)*feval(hfx, X(:,1), tpara);  m = length(y);
    Y = repmat(y,1,2*n);
    for k=2:1:n+2
        Y(:,k) = feval(hfx, X(:,k), tpara);
        y = y + SSUT_w(k)*Y(:,k);
    end
    Pyy = zeros(m); Pxy = zeros(n,m);
    for k=1:1:n+2
        yerr = Y(:,k)-y;
        Pyy = Pyy + SSUT_w(k)*(yerr*yerr');  % variance
        xerr = X(:,k)-x;
        Pxy = Pxy + SSUT_w(k)*xerr*yerr';  % covariance
    end
    