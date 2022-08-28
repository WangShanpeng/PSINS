function [X1, Phik, D] = vfbfx(X0, tpara)
% Vertically falling body state equation f(x).
% Ref: Athans, M. 'Suboptimal state estimation for continuous-time nonlinear systems 
% from discrete noisy measurements'. IEEE Transactions on Automatic Control,1968.
% [x1,x2,x3]: altitude,velocity,ballistic-parameter, state equations:
%   dx1/dt = -x2
%   dx2/dt = -exp(-gamma*x1)*x2^2*x3
%   dx3/dt = 0

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/08/2022
    gamma = tpara.gamma; ts = tpara.ts;
    x1 = X0(1); x2 = X0(2); x3 = X0(3);
    %% X0->X1
%     X1 = [ x1 - x2*ts;
%            x2 - exp(-gamma*x1)*x2^2*x3*ts;
%            x3 ];
    k1 = [-X0(2);  -exp(-gamma*X0(1))*X0(2)^2*X0(3);    0];  X01 = X0+k1*ts/2;
    k2 = [-X01(2); -exp(-gamma*X01(1))*X01(2)^2*X01(3); 0];  X02 = X0+k2*ts/2;
    k3 = [-X02(2); -exp(-gamma*X02(1))*X02(2)^2*X02(3); 0];  X03 = X0+k3*ts;
    k4 = [-X03(2); -exp(-gamma*X03(1))*X03(2)^2*X03(3); 0];
    X1 = X0 + ts/6*(k1+2*(k2+k3)+k4);
    %% Jacobian Phik
    if nargout>1
        Phik = [0,                             -1,                      0;
               gamma*exp(-gamma*x1)*x2^2*x3,  -exp(-gamma*x1)*2*x2*x3, -exp(-gamma*x1)*x2^2;
               0,                             0,                       0 ];
        Phik = eye(size(Phik)) + Phik*ts;   % Phi = expm(Phi*ts);
    end
    %% Hessian D
    if nargout>2
        egx1 = exp(-gamma*x1);
        D{1} = zeros(3);
        D{2} = [ -gamma^2*egx1*x2^2*x3, gamma*egx1*2*x2*x3, gamma*egx1*x2^2
                 0, -egx1*2*x3, -egx1*2*x2
                 0, 0, 0 ]*ts;       D{2}(2,1)=D{2}(1,2); D{2}(3,1)=D{2}(1,3); D{2}(3,2)=D{2}(2,3);
        D{3} = zeros(3);
    end

