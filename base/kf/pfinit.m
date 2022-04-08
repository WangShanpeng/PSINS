function pf = pfinit(x0, P0, Qk, Rk, N)
% Particle filter structure initialization.
%
% Prototype: pf = pfinit(x0, P0, Qk, Rk, N)
% Inputs: x0,P0 - initial state & covariance;
%         Qk,Rk - process & measure covariance
%         N - particle number
% Output: pf - particle filter structure array
%
% see also  pfupdate, kfinit.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/04/2022
    if size(P0,2)>1, P0 = diag(P0); end
    pf.n = length(x0);  pf.m = length(Rk);
    for k=1:pf.n
        pf.particles(k,:) = x0(k)+sqrt(P0(k))*randn(1,N);
    end
    pf.Npts = N;
	pf.Pxk = P0; pf.Qk = Qk; pf.Rk = Rk;

    