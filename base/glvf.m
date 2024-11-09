function glv1 = glvf(Re, f, wie)
% PSINS Toolbox global variable structure initialization.
%
% Prototype: glv = glvf(Re, f, wie)
% Inputs: Re - the Earth's semi-major axis
%         f - flattening
%         wie - the Earth's angular rate
% Output: glv1 - output global variable structure array
%
% See also  psinsinit.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 14/08/2011, 10/09/2013, 09/03/2014
global glv
    if ~exist('Re', 'var'),  Re = [];  end
    if ~exist('f', 'var'),   f = [];  end
    if ~exist('wie', 'var'), wie = [];  end
    if isempty(Re),  Re = 6378137;  end
    if isempty(f),   f = 1/298.257;  end
    if isempty(wie), wie = 7.2921151467e-5;  end
    glv.Re = Re;                    % the Earth's semi-major axis
    glv.f = f;                      % flattening
    glv.Rp = (1-glv.f)*glv.Re;      % semi-minor axis
    glv.e = sqrt(2*glv.f-glv.f^2); glv.e2 = glv.e^2; % 1st eccentricity
    glv.ep = sqrt(glv.Re^2-glv.Rp^2)/glv.Rp; glv.ep2 = glv.ep^2; % 2nd eccentricity
    glv.GM = 3.986004418e14;        % Earth's universal
    glv.wie = wie;                  % the Earth's angular rate
    glv.meru = glv.wie/1000;        % milli earth rate unit
    glv.g0 = 9.7803267715;          % gravitational force
    m = Re*glv.wie^2/glv.g0;  glv.beta = 5/2*m-f-17/14*m*f;
    glv.beta1 = (5*m*f-f^2)/8; glv.beta2 = 3.086e-6; glv.beta3 = 8.08e-9;
    glv.mg = 1.0e-3*glv.g0;         % milli g
    glv.ug = 1.0e-6*glv.g0;         % micro g
    glv.mGal = 1.0e-3*0.01;         % milli Gal = 1cm/s^2 ~= 1.0E-6*g0
    glv.uGal = glv.mGal/1000;       % micro Gal
    glv.ugpg = glv.ug/glv.g0;       % ug/g
    glv.ugpg2 = glv.ug/glv.g0^2;    % ug/g^2
    glv.ugpg3 = glv.ug/glv.g0^3;    % ug/g^3
    glv.ws = 1/sqrt(glv.Re/glv.g0); % Schuler frequency
    glv.ppm = 1.0e-6;               % parts per million
    glv.deg = pi/180;               % arcdeg
    glv.min = glv.deg/60;           % arcmin
    glv.sec = glv.min/60;           % arcsec
    glv.mas = glv.sec/1000;         % milli arcsec
    glv.hur = 3600;                 % time hour (1hur=3600second)
    glv.dps = pi/180/1;             % arcdeg / second
    glv.mdps = glv.dps/1000;        % milli dps
    glv.rps = 360*glv.dps;          % revolutions per second
    glv.rpm = 360*glv.dps/60;       % revolutions per min
    glv.dph = glv.deg/glv.hur;      % arcdeg / hour
    glv.dpss = glv.deg/sqrt(1);     % arcdeg / sqrt(second)
    glv.dpsh = glv.deg/sqrt(glv.hur);  % arcdeg / sqrt(hour)
    glv.dphpsh = glv.dph/sqrt(glv.hur); % (arcdeg/hour) / sqrt(hour)
    glv.dph2 = glv.dph/glv.hur;    % (arcdeg/hour) / hour
    glv.secpg = glv.sec/glv.g0;    % arcsec / g
    glv.secpdps2 = glv.sec/(glv.deg/1^2);    % arcsec / (deg/s^2)
    glv.secprps2 = glv.sec/(1/1^2);    % arcsec / (rad/s^2)
    glv.Hz = 1/1;                   % Hertz
    glv.dphpsHz = glv.dph/glv.Hz;   % (arcdeg/hour) / sqrt(Hz)
    glv.dphpg = glv.dph/glv.g0;     % (arcdeg/hour) / g
    glv.dphpg2 = glv.dphpg/glv.g0;  % (arcdeg/hour) / g^2
    glv.ugpsHz = glv.ug/sqrt(glv.Hz);  % ug / sqrt(Hz)
    glv.mpsph = 1/sqrt(glv.hur);          % m/s/sqrt(hour)
    glv.ugpsh = glv.ug/sqrt(glv.hur); % ug / sqrt(hour)
    glv.ugph = glv.ug/glv.hur;      % ug / hour
    glv.ugphpsh = glv.ugph/sqrt(glv.hur);  % ug / hour /sqrt(hur)
    glv.gxs = glv.g0*1;             % g0*s
    glv.mpsh = 1/sqrt(glv.hur);     % m / sqrt(hour)
    glv.mpspsh = 1/1/sqrt(glv.hur); % (m/s) / sqrt(hour), 1*mpspsh~=1700*ugpsHz
    glv.ppmpsh = glv.ppm/sqrt(glv.hur); % ppm / sqrt(hour)
    glv.mil = 2*pi/6000;            % mil
    glv.nm = 1853;                  % nautical mile
    glv.kn = glv.nm/glv.hur;        % knot
    glv.kmph = 1000/glv.hur;        % km/hour
    %%
    glv.wm_1 = [0,0,0]; glv.vm_1 = [0,0,0];   % the init of previous gyro & acc sample
    glv.cs = [                      % coning & sculling compensation coefficients
        [2,    0,    0,    0,    0    ]/3
        [9,    27,   0,    0,    0    ]/20
        [54,   92,   214,  0,    0    ]/105
        [250,  525,  650,  1375, 0    ]/504 
        [2315, 4558, 7296, 7834, 15797]/4620 ];
    glv.csmax = size(glv.cs,1)+1;  % max subsample number
    glv.csCompensate = 1; % csCompensate=0 or 1, default 1
    glv.v0 = [0;0;0];    % 3x1 zero-vector
    glv.qI = [1;0;0;0];  % identity quaternion
    glv.I33 = eye(3); glv.o33 = zeros(3);  % identity & zero 3x3 matrices
%   glv.pos0 = [34.246048*glv.deg; 108.909664*glv.deg; 380]; % position of INS Lab@NWPU old
    glv.pos0 = [34.034310*glv.deg; 108.775427*glv.deg; 450];
    glv.eth = []; glv.eth = earth(glv.pos0);
    glv.t0 = 0;
    glv.tscale = 1;  % =1 for second, =60 for minute, =3600 for hour, =24*3600 for day
    glv.isfig = 1;
    glv.gfix = []; glv.dgn = [];
    %%
    [glv.rootpath, glv.datapath, glv.mytestflag] = psinsenvi;
    glv1 = glv;

