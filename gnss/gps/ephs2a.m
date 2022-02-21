function epha = ephs2a(ephs)
% Translate ephemeris structure to array
%
% Prototype: epha = epha2s(ephs)
% Inputs: ephs - ephemeris data in structure format
% Outputs: epha - ephemeris data in array format
%
% See also  epha2s, findEph, lsPos.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/01/2022
    m = length(ephs);
    epha = zeros(m,30);
    for k=1:m
        eph = ephs(k,1);
        epha(k,:) = [ eph.PRN, ...
            eph.Toc,eph.af0,eph.af1,eph.af2, eph.IODE,eph.Crs,eph.Deltan,eph.M0, ...
            eph.Cuc,eph.e,eph.Cus,eph.sqrtA,  eph.Toe,eph.Cic,eph.OMEGA0,eph.Cis, ...
            eph.i0,eph.Crc,eph.omega,eph.OMEGADot, eph.iDot,eph.L2Cd,eph.WN,eph.L2PF, ...
            eph.URA,eph.SatHl,eph.TGD,eph.SOW,eph.FitI ];
    end
    