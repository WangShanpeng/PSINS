function [X1, Phik, D] = vfbfx(X0, tpara)
% Vertically falling body state equation f(x).
% Ref: Athans, M. 'Suboptimal state estimation for continuous-time nonlinear systems 
% from discrete noisy measurements'. IEEE Transactions on Automatic Control,1968.
% [x1,x2,x3]: altitude,velocity,ballistic-parameter, state equations:
%   dx1/dt = -x2
%   dx2/dt = g-exp(-gamma*x1)*x2^2*x3
%   dx3/dt = 0
%
% See also  vfbhx.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/08/2022
    %     Phik = eye(3) + [0,1,0; 0,0,1; 0,0,0]*tpara.ts; D{1}=zeros(3); D{2}=zeros(3); D{3}=zeros(3);  % linear model test
    %     X1 = Phik*X0;
    %     return;  % for linear test
    gamma = tpara.gamma;  g = tpara.g;  Ts = tpara.Ts;
    x1 = X0(1); x2 = X0(2); x3 = X0(3);
    %% X0->X1
    egx1 = exp(-gamma*x1);
    X1 = [ x1 - x2*Ts;   % Euler1 discretization
           x2 + (g-egx1*x2^2*x3)*Ts;
           x3 ];
%     k1 = [-X0(2);  g-exp(-gamma*X0(1))*X0(2)^2*X0(3);    0];  X01 = X0+k1*Ts/2;  % RK4
%     k2 = [-X01(2); g-exp(-gamma*X01(1))*X01(2)^2*X01(3); 0];  X02 = X0+k2*Ts/2;
%     k3 = [-X02(2); g-exp(-gamma*X02(1))*X02(2)^2*X02(3); 0];  X03 = X0+k3*Ts;
%     k4 = [-X03(2); g-exp(-gamma*X03(1))*X03(2)^2*X03(3); 0];
%     X1 = X0 + Ts/6*(k1+2*(k2+k3)+k4);
    %% Jacobian Phik
    if nargout>1
        Phik = [ 0,            -1,    0;
                [gamma*x2*x3,  -2*x3, -x2]*egx1*x2;
                 0,            0,     0 ];
        Phik = eye(size(Phik)) + Phik*Ts;   % Phi = expm(Phi*ts);
    end
    %% Hessian D
    if nargout>2
        D{1} = zeros(3);
        D{2} = [ [-gamma*x2*x3, 2*x3,       x2]*gamma*egx1*x2;
                  0,            -2*egx1*x3, -2*egx1*x2
                  0,            0,          0 ]*Ts;
        D{2}(2,1)=D{2}(1,2); D{2}(3,1)=D{2}(1,3); D{2}(3,2)=D{2}(2,3); % symmetric
        D{3} = zeros(3);
    end

