% global value settings for GLONASS positioning simulation.
% See also  glvs, ggpsvars, gbdvars.
% Copyright(c) 2009-2015, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/08/2015
clear global gglo
global gglo
gglo.c           = 299792458;
gglo.wie         = 7.2921151467e-5;  % Earth rotation rate, [rad/s]
gglo.GM          = 3.986004418e14;   % Earth's universal
gglo.F           = -4.442807633e-10; % Constant, [sec/(meter)^(1/2)]
gglo.a           = 6378136;          % WGS-84 semi-major axis
gglo.J02         = 1.0826257e-3;     %
gglo.f           = 1/298.257;        % flattening
gglo.fCA         = 5.0e6;            % C/A code frequency
gglo.fL1         = 1602.0e6+(-7:6)'*562.5e3;  % L1 carrier frequency
gglo.fL2         = 1246.0e6+(-7:6)'*437.5e3;  % L2 carrier frequency
gglo.lambda1     = gglo.c./gglo.fL1;    % L1 carrier wavelength
gglo.lambda2     = gglo.c./gglo.fL2;    % L1 carrier wavelength
gglo.recpos = [ones(3,1);0];  % receiver init position & clock bias [x,y,z,dt]
gglo.ephs = struct( 'PRN', 1, ...
                    'Toc', 2, 'tauN',  3, 'gammaN', 4, 'tk',     5, ...
                    'X',   6, 'Xdot',  7, 'Xdd',    8, 'satHl',  9, ...
                    'Y',  10, 'Ydot', 11, 'Ydd',   12, 'frqNO', 13, ...
                    'Z',  14, 'Zdot', 15, 'Zdd',   16, 'day',   17 );
glvs
