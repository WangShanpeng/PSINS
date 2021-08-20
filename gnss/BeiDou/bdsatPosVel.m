function [satPoss, satClkCorrs, satVels] = bdsatPosVel(transmitTime, eph)
% Calculate satellite position(s), clock error(s) and velocity(s) from ephemeris data.
%
% Prototype: [satPoss, satClkCorrs, satVels] = bdSatPosVel(transmitTime, eph) 
% Inputs: transmitTime - satellite signal transmission time
%         eph - ephemeris data
% Outputs: satPoss - satellite positions in ECEF at transmit time
%          satClkCorrs - satellite clock corrections
%
% See also  findEph, lsPos.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/07/2015
    numOfSatellites = size(transmitTime, 1);
    satPoss         = zeros(3, numOfSatellites); satVels = satPoss;
    satClkCorrs     = zeros(numOfSatellites, 1);
    for k=1:numOfSatellites
        if nargout==3
            [satPoss(:,k), satClkCorrs(k)] = bdsatpv(transmitTime(k), eph(k,:));
            dt = 0.001;
            satPoss0 = bdsatpv(transmitTime(k)-dt, eph(k,:));
            satPoss1 = bdsatpv(transmitTime(k)+dt, eph(k,:));
            satVels(:,k) = (satPoss1-satPoss0)/dt/2;
        else
            [satPoss(:,k), satClkCorrs(k)] = bdsatpv(transmitTime(k), eph(k,:));
        end
    end
    
function [satPos, satClkCorr] = bdsatpv(transmitTime, eph)
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/07/2015
global gbd
    dt = transmitTime - eph.Toc;
    if dt>302400, dt=dt-604800; elseif dt<-302400, dt=dt+604800;  end
    satClkCorr = eph.af0 + eph.af1*dt+eph.af2*dt*dt - 0*eph.TGD1;   % 钟差改正（0.1ms量级）
    
    tk = transmitTime - satClkCorr - eph.Toe;                    % 归化时间
    if tk>302400, tk=tk-604800; elseif tk<-302400, tk=tk+604800;  end

    a   = eph.sqrtA * eph.sqrtA;   % 长半轴
    n0  = sqrt(gbd.GM / a^3);   % 平均角速率
    n   = n0 + eph.Deltan;   % 摄动改正

    M   = eph.M0 + n*tk;   % 平近点角
    E   = M;  dE  = 1.0;    % 偏近点角
    while abs(dE) > 1.e-12
        E_old   = E;
        sinE    = sin(E);
        E       = M + eph.e*sinE;
        dE      = E-E_old;
    end
    cosE = cos(E);
    nu   = atan2(sqrt(1-eph.e^2)*sinE, cosE-eph.e);  % 真近点角
    
    phi  = nu + eph.omega;   % 升交角距
    sin2phi = sin(2*phi); cos2phi = cos(2*phi);

    u = phi +                   eph.Cuc*cos2phi + eph.Cus*sin2phi;  % 升交角距改正
    r = a*(1-eph.e*cosE) +      eph.Crc*cos2phi + eph.Crs*sin2phi;  % 失径及改正
    i = eph.i0 + eph.iDot*tk +  eph.Cic*cos2phi + eph.Cis*sin2phi;  % 轨道倾角及改正

    sinu = sin(u); cosu = cos(u);
    sini = sin(i); cosi = cos(i);
    x = r*cosu; y = r*sinu;  % 卫星在轨道平面中坐标（注意：X轴指向升交点，而不是近地点）
    if eph.PRN<=505  % satellite is GEO
        Omega = eph.OMEGA0+eph.OMEGADot*tk-gbd.wie*eph.Toe;
        sinOmega = sin(Omega); cosOmega = cos(Omega);
        xyz = [ x*cosOmega - y*cosi*sinOmega; x*sinOmega + y*cosi*cosOmega; y*sini ];
        wtk = gbd.wie*tk;
        swtk = sin(wtk);    cwtk = cos(wtk);
        s5 = sin(-5*pi/180);  c5 = cos(-5*pi/180);
        satPos = [ cwtk, swtk*c5, swtk*s5;
                  -swtk, cwtk*c5, cwtk*s5;
                   0    -s5       c5 ] * xyz;
    else  % is IGSO or MEO
        Omega = eph.OMEGA0 + (eph.OMEGADot-gbd.wie)*tk - gbd.wie*eph.Toe; % 升交点经度
        sinOmega = sin(Omega); cosOmega = cos(Omega);
        satPos = [ x*cosOmega - y*cosi*sinOmega; x*sinOmega + y*cosi*cosOmega; y*sini ];  % ECEF坐标
    end
    
    satClkCorr = satClkCorr + gbd.F*eph.e*eph.sqrtA*sinE;   % 钟差相对论改正（10ns量级，对卫星位置影响小）
