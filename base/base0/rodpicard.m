function rod = rodpicard(Wt, T, iter0)
% Solution for Rodrigues parameters differential equation (Picard serial).
% 
% Prototype: rod = rodpicard(Wt, T, iter0)
% Inputs: Wt - 3xn angluar rate coefficients of the polynomial in 
%              descending powers
%         T - one step forward from time 0 to T
%         iter0 - iteration count
% Outputs: rod - output Rodrigues parameters
%
% see also  qpicard, btzpicard.

% Copyright(c) 2009-2019, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/10/2018
    if nargin<3, iter0 = 10; end
    if iter0>20, iter0 = 20; end
    rod = [0;0;0];
    for iter=1:iter0
        pw = polycross(rod,Wt);
        pdotw = polydot(rod,Wt);
        rod = polycut(polyintn(polyadd(Wt, 0.5*pw, 0.25*polydotmul(pdotw,rod))), 20);
    end
    rod = polyvaln(rod, T);
