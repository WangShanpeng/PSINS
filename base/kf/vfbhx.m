function [Z, Hk, D] = vfbhx(X, tpara, rk)
% Vertically falling body measurement equation h(x).
% Ref: Athans, M. 'Suboptimal state estimation for continuous-time nonlinear systems 
% from discrete noisy measurements'. IEEE Transactions on Automatic Control,1968.
% [x1,x2,x3]: altitude,velocity,ballistic-parameter, measurement equation:
%   Z = sqrt(M^2+(x3-H)^2) + V
%
% See also  vfbfx.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/08/2022
    %     Z = X(1);  Hk = [1, 0, 0];  D{1} = zeros(3);   % linear model test
    %     if nargin==3, Z=Z+V*randn(1); end
    %     return;  % for linear test
    M = tpara.M; H = tpara.H;
    %% Z
    Z = sqrt(M^2+(X(1)-H)^2);
    %% Jacobian Hk
    if nargout>1,  Hk = [(X(1)-H)/Z, 0, 0];  end
    %% Hessian D
    if nargout>2,  D{1} = [M^2/Z^3, 0, 0; 0, 0, 0; 0, 0, 0];  end
    %% Measurement noise
    if nargin==3, Z=Z+rk*randn(1); end

