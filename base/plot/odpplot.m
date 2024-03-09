function odpplot(odp)
% Odometer parameter plot.
%
% Prototype: odpplot(odp)
% Input: odp - Odometer parameter = [dpitch, Kod, dyaw, t] array.
%
% See also  odplot, lvtplot, xpplot, igoplot.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/07/2021
global glv
    myfig
	subplot(211), plot(odp(:,end), odp(:,[1,3])/glv.deg); xygo('\it\delta\theta,\delta\psi\rm / \circ'); 
	subplot(212), plot(odp(:,end), odp(:,2)); xygo('Kod');