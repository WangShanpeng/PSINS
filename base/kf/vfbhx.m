function [Z, Hk, D] = vfbhx(X, tpara)
% Vertically falling body measurement equation h(x).
% Ref: Athans, M. 'Suboptimal state estimation for continuous-time nonlinear systems 
% from discrete noisy measurements'. IEEE Transactions on Automatic Control,1968.
% [x1,x2,x3]: altitude,velocity,ballistic-parameter, measurement equation:
%   Z = sqrt(M^2+(x3-H)^2) + v

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/08/2022
    M = tpara.M; H = tpara.H;
    %% Z
    Z = sqrt(M^2+(X(1)-H)^2);
    %% Jacobian Hk
    if nargout>1
        Hk = [(X(1)-H)/Z, 0, 0];
    end
    %% Hessian D
    if nargout>2
        D{1} = [1/Z-(X(1)-H)^2/Z^3, 0, 0; 0, 0, 0; 0, 0, 0];
    end

