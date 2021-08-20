function [satPosVel, satClkCorr, TGD, orbitp] = bdsatPosVelBatch(transmitTime, eph)
% Calculate satellite position(s), clock error(s) and velocity(s) from ephemeris data
% using batch matrix processing method.
%
% Prototype: [satPosVel, satClkCorr, TGD, orbitp] = bdsatPosVelBatch(transmitTime, eph)
% Inputs: transmitTime - satellite signal transmission time
%         eph - ephemeris data
% Outputs: satPosVel - satellite positions/velocities in ECEF at transmit time
%          satClkCorrs - satellite clock corrections
%          TGD - TGD
%          orbitp - other orbit parameters
%
% See also

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/07/2015
    dt = 0.001;
    [satPos, satClkCorr, TGD, orbitp] = bdsatPosBatch(transmitTime, eph);
    satPos0 = bdsatPosBatch(transmitTime-dt/2, eph);
    satPos1 = bdsatPosBatch(transmitTime+dt/2, eph);
    satPosVel = [satPos, (satPos1-satPos0)/dt];
