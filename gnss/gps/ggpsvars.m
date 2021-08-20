% global value settings for GPS positioning simulation.
% See also  glvs, gbdvars, glovars.
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/08/2013
clear global ggps
global ggps
ggps.c           = 299792458;
ggps.wie         = 7.2921151467e-5;  % Earth rotation rate, [rad/s]
ggps.GM          = 3.986004418e14;   % Earth's universal
ggps.F           = -4.442807633e-10; % Constant, [sec/(meter)^(1/2)]
ggps.a           = 6378137;          % WGS-84 semi-major axis
ggps.f           = 1/298.257223563;  % flattening
ggps.fCA         = 1.023e6;          % C/A code frequency
ggps.fL1         = 1540*ggps.fCA;    % L1 carrier frequency
ggps.fL2         = 1200*ggps.fCA;    % L2 carrier frequency
ggps.lambda1     = ggps.c/ggps.fL1;  % L1 carrier wavelength
ggps.lambda2     = ggps.c/ggps.fL2;  % L2 carrier wavelength
ggps.kf          = (ggps.fL1/ggps.fL2)^2;
ggps.recpos = [ones(3,1);0];  % receiver init position & clock bias [x,y,z,dt]
ggps.ephs = struct( 'PRN', 1, ...
        'Toc', 2, 'af0',   3, 'af1',   4, 'af2',      5,  'IODE', 6, 'Crs',  7, 'Deltan', 8, 'M0',   9, ...
        'Cuc',10, 'e',    11, 'Cus',  12, 'sqrtA',   13,  'Toe', 14, 'Cic', 15, 'OMEGA0',16, 'Cis', 17, ...
        'i0', 18, 'Crc',  19, 'omega',20, 'OMEGADot',21,  'iDot',22, 'L2Cd',23, 'WN',    24, 'L2PF',25, ...
        'URA',26, 'SatHl',27, 'TGD',  28, 'IODC',    29,  'SOW', 30, 'FitI',31 );
glvs
