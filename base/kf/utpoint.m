function [U, wm, wc, gamma] = utpoint(n, afa, beta, kappa)
% Calculate 3th or 5th-order cubature points.
%
% Prototype: [U, wm, wc] = utpoint(n, afa, beta, kappa)
% Inputs: n - dimension
%         afa,beta,kappa - parameters
% Outputs: U - UT points
%          wm,wc - weights
%         gamma - gamma parameter
%
% See also  cubpoint, ghpoint, ukf.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/08/2022
    if nargin<4, kappa=0; end
    if nargin<3, beta=2; end
    if nargin<2, afa=1e-3; end
    lambda = afa^2*(n+kappa)-n;  gamma = sqrt(n+lambda);
    U = gamma*[eye(n), -eye(n), zeros(n,1)];
    wm = [repmat(1/(2*gamma^2),1,2*n), lambda/gamma^2];
    wc = wm; wc(end) = wc(end)+(1-afa^2+beta);

    