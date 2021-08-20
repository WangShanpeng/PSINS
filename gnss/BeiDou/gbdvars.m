% global value settings for GPS positioning simulation.
% See also  glvs, ggpsvars, gglovars.
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/08/2015
clear global gbd
global gbd
gbd.c           = 299792458;
gbd.wie         = 7.2921150e-5;     % Earth rotation rate, [rad/s]
gbd.GM          = 3.986004418e14;   % Earth's universal
gbd.F           = -4.442807633e-10; % Constant, [sec/(meter)^(1/2)]
gbd.a           = 6378137;          % WGS-84 semi-major axis
gbd.f           = 1/298.257222101;  % flattening
gbd.fCA         = 2.046e6;          % C/A code frequency
gbd.fL1         = 1561.098e6;       % L1 carrier frequency
gbd.fL2         = 1207.140e6;       % L2 carrier frequency
gbd.fL3         = 1268.520e6;       % L3 carrier frequency
gbd.lambda1     = gbd.c/gbd.fL1;    % L1 carrier wavelength
gbd.lambda2     = gbd.c/gbd.fL2;    % L2 carrier wavelength
gbd.lambda3     = gbd.c/gbd.fL3;    % L3 carrier wavelength
gbd.kf          = (gbd.fL1/gbd.fL2)^2;
gbd.XYZt = [ones(3,1);0];  % receiver init position & clock bias [x,y,z,dt]
gbd.BLH = zeros(3,1);
gbd.ephs = struct( 'PRN', 1, ...
        'Toc', 2, 'af0',   3, 'af1',   4, 'af2',      5,  'IODE', 6, 'Crs',  7, 'Deltan', 8, 'M0',   9, ...
        'Cuc',10, 'e',    11, 'Cus',  12, 'sqrtA',   13,  'Toe', 14, 'Cic', 15, 'OMEGA0',16, 'Cis', 17, ...
        'i0', 18, 'Crc',  19, 'omega',20, 'OMEGADot',21,  'iDot',22, 'NA23',23, 'WN',    24, 'NA25',25, ...
        'URA',26, 'SatHl',27, 'TGD1', 28, 'TGD2',    29,  'SOW', 30, 'IODC',31 );
ggpsvars;
