function [satPosVel, satClkCorr, TGD, orbitp] = satPosVelBatch(transmitTime, eph)
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/08/2013, 30/06/2015
    dt = 0.001;
    [satPos, satClkCorr, TGD, orbitp] = satPosBatch(transmitTime, eph);
    satPos0 = satPosBatch(transmitTime-dt/2, eph);
    satPos1 = satPosBatch(transmitTime+dt/2, eph);
    satPosVel = [satPos, (satPos1-satPos0)/dt];
