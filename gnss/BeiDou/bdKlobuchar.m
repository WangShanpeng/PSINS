function dion = bdKlobuchar(coef, BL, AzEl, tp)
% Ionospheric correction using Klobuchar model.(Ref. BDS-SIS-ICD-2.0 pp23)
%
% Prototype: drho = bdKlobuchar(afabeta, BL, AzEl, tp)
% Inputs: coef - ION coefficience = [alpha0~3, beta0~3]
%         BL - receiver's geographical coordinate [latitude, longitude]
%         AzEl - satellite azimuth and elevation
%         tp - observation time
% Output: dion - pseudorange correction (in meter) for B1I
%
% See also  topocent.

% Copyright(c) 2009-2015, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 24/07/2015
    R = 6378e3; h = 375e3; c = 3e8;
    alpha = coef(:,1); beta = coef(:,2);
    phiu = BL(1); lambdau = BL(2); Az = AzEl(:,1); El = AzEl(:,2);
    len = size(Az,1); dion = zeros(len,1);
    for k=1:len
        RRhcE = R/(R+h)*cos(El(k));
        psi = pi/2-El(k)-asin(RRhcE);
        phiM = asin(sin(phiu)*cos(psi)+cos(phiu)*sin(psi)*cos(Az(k)))/pi;
        lambdaM = lambdau + asin(sin(psi)*sin(Az(k))/cos(phiM));
        phi0_3 = abs(phiM).^(0:3)';
        A2 = alpha'*phi0_3;  if A2<0, A2=0; end
        A4 = beta' *phi0_3;  if A4>=172800, A4=172800; elseif A4<72000, A4=72000; end
        t = mod(tp+lambdaM/(2*pi)*86400,86400);  % local time at lambdaM
        dt = t - 50400;
        Iz = 5e-9;
        if abs(dt)<A4/4, Iz = Iz + A2*cos(2*pi*dt/A4); end
        IB1I = Iz/sqrt(1-RRhcE^2);
        dion(k) = IB1I*c;	% for B2I *(1561/1207)^2
    end
