function ephs = epha2s(epha)
% Translate ephemeris array to structure
%
% Prototype: ephs = epha2s(epha)
% Inputs: epha - ephemeris data in array format
% Outputs: ephs - ephemeris data in structure format
%
% See also  ephs2a, findEph, lsPos.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/01/2022
    global ggps;
    [m, n] = size(epha);
    ephs(m,1) = ggps.ephs;  eph = ggps.ephs;
    for k=1:m
        eph.PRN = epha(k,1);
        eph.Toc = epha(k,2);    eph.af0 = epha(k,3);     eph.af1 = epha(k,4);      eph.af2 = epha(k,5);
        eph.IODE = epha(k,6);   eph.Crs = epha(k,7);     eph.Deltan = epha(k,8);   eph.M0 = epha(k,9);
        eph.Cuc = epha(k,10);   eph.e = epha(k,11);      eph.Cus = epha(k,12);     eph.sqrtA = epha(k,13);
        eph.Toe = epha(k,14);   eph.Cic = epha(k,15);    eph.OMEGA0 = epha(k,16);  eph.Cis = epha(k,17);
        eph.i0 = epha(k,18);    eph.Crc = epha(k,19);    eph.omega = epha(k,20);   eph.OMEGADot = epha(k,21);
        eph.iDot = epha(k,22);  eph.L2Cd = epha(k,23);   eph.WN = epha(k,24);      eph.L2PF = epha(k,25);
        eph.URA = epha(k,26);   eph.SatHl = epha(k,27);  eph.TGD = epha(k,28);     eph.SOW = epha(k,29);
        eph.FitI = epha(k,30);
        ephs(k,1) = eph;
    end
    