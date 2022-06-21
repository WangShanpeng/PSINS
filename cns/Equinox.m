function [eps, dpsi, deps] = Equinox(T)
% Equinox parameter calculation.
% Ref. 'Space-time coordinate transformation and its error analysis for
%      INS/CNS integrated navigation, 2022'
%
% Prototype: [eps, dpsi, deps] = Equinox(T)
% Input: T - using TDB to predict the precession
% Outputs: eps, dpsi, deps - equinox parameters in arcsec
%
% See also  GAST, Nutmat.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/11/2021
global glv
	T2 = T^2; T3 = T^3;
    eps  = (84381.448-46.8150*T-0.00059*T2+0.001813*T3)*glv.sec;
    lp = ( 0.000136*T3-0.5532*T2+129596581.0481*T+1287104.79305)*glv.sec;
    F  = (-0.001037*T3-12.7512*T2+1739527262.8478*T+335779.526232)*glv.sec;
    D  = ( 0.006593*T3-6.3706*T2+1602961601.2090*T+1072260.70369)*glv.sec;
    Om = ( 0.007702*T3+7.4722*T2-6962890.5431*T+450160.398036 )*glv.sec;
    dpsi = ( (-17.1996+0.01742*T)*sin(Om)     +(0.2062+0.00002*T)*sin(2*Om)...
            -(1.3187+0.00016*T)*sin(2*F-2*D+2*Om)    +(0.1426-0.00034*T)*sin(lp)...
            -(0.2274+0.00002*T)*sin(2*F+2*Om) )*glv.sec;
    deps = ( (9.2025+0.00089*T)*cos(Om)    -(0.0895-0.00005*T)*cos(2*Om)...
            +(0.5736-0.00031*T)*cos(2*F-2*D+2*Om)   +(0.0054-0.00001*T)*cos(lp)...
            +(0.0977-0.00005*T)*cos(2*F+2*Om) )*glv.sec;

