function satVel = satVelCorr(satVel, rho)
% Satellite velocities correction from transmit ECEF to reception ECEF, 
% but this effect is very small (in cm/s level).
%
% Prototype: satVel = satVelCorr(satVel, rho) 
% Inputs: satVel - satellite velocities in ECEF at transmission time 
%         rho - distances between receiver and satellites
% Output: satVel - satellite velocities in ECEF at reception time
%
% See also  satPosVel, rhoSatRec, lsVel.

% Copyright(c) 2009-2015, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/07/2015
global ggps
    wtau = ggps.wie * rho/ggps.c;
    sw = sin(wtau);  cw = cos(wtau);
    satVel = [cw.*satVel(:,1)+sw.*satVel(:,2), -sw.*satVel(:,1)+cw.*satVel(:,2), satVel(:,3)]; 

