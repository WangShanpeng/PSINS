function [pv1, clkerr] = glosatPosVel(transmitTime, eph)
% Calculate GLONASS satellite position(s), clock error(s) and velocity(s) 
% from ephemeris data.
%
% Prototype: [pv1, clkerr] = glosatPosVel(transmitTime, eph) 
% Inputs: transmitTime - satellite signal transmission time
%         eph - ephemeris data
% Outputs: pv1 - satellite positions/velocities in ECEF at transmit time
%          clkerr - satellite clock corrections
%
% See also  satPosVel, bdsatPosVel.

% Copyright(c) 2009-2015, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/08/2015
    tb = eph(:,2);  tauN = eph(:,3);  gammaN = eph(:,4);
    dt = transmitTime-tb;
    if dt>302400, dt=dt-604800; elseif dt<-302400, dt=dt+604800;  end
    clkerr = -gammaN.*(transmitTime-tb) + tauN;
    tk = transmitTime - clkerr - tb;
    if tk>302400, tk=tk-604800; elseif tk<-302400, tk=tk+604800;  end
    pv0 = eph(:,[6,10,14,7,11,15])*1e3; a0 = eph(:,[8,12,16])*1e3;
    kk = fix(max(abs(tk))/30)+1;
    for k=1:kk, pv0 = satpvRgkt(pv0, a0, tk/kk); end
    pv1 = pv0;
    
function pv1 = satpvRgkt(pv0, a0, dt)
% Runge-Kutta method
    dt = repmat(dt,1,6);
    k1 = deq(pv0,          a0);
    k2 = deq(pv0+dt/2.*k1, a0);
    k3 = deq(pv0+dt/2.*k2, a0);
    k4 = deq(pv0+dt.*k3,   a0);
    pv1 = pv0+dt/6.*(k1+2*(k2+k3)+k4);
    
function ki = deq(pv, a0)
% Ref. RTKLIB 2.4.2 / 'deq' C-function
    u = 3.9860044e14; ae2 = 6378136^2; wie = 7.292115e-5; J02=1.0826257e-3;
    r2 = sum(pv(:,1:3).^2,2); r3 = r2.*sqrt(r2); r5 = r2.*r3;
    a = 3/2*J02*u*ae2./r5;
    b = 5*pv(:,3).^2./r2;
    c = -u./r3 - a.*(1-b);
    ki = [ pv(:,4:6), [(c+wie^2).*pv(:,1)+2*wie*pv(:,5),(c+wie^2).*pv(:,2)-2*wie*pv(:,4),(c-2*a).*pv(:,3)]+a0 ];
