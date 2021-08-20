function [recVel, errVel] = lsVel(satVel, LOS, W, PRDot, rho)
% Calculate receiver's velocity using least square method.
%
% Prototype: [recPos, rho, res] = lsPos(satPos, recPos, rho, obs) 
% Inputs: satVel - satellite velocities in ECEF at transmission time 
%         LOS - satellite line-of-sight vectors in ECEF at reception time
%         W - weight matrix
%         PRDot - Doppler observations in meter
%         rho - distances between receiver and satellites, for satVel correction
% Output: recVel - receiver velocity = [Vx, Vy, Vz, clock-dirft in meter/s]
%
% See also  lsPos.

% Copyright(c) 2009-2015, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/07/2015
    if nargin==5, satVel = satVelCorr(satVel, rho); end
    A = [LOS, ones(size(LOS(:,1)))];
    y = PRDot + dot(LOS,satVel,2);
    recVel = (A'*W*A)^-1*A'*W* y;
    err = sqrt(W)*(y - A*recVel); errVel = sqrt(err'*err);  % LS residual error