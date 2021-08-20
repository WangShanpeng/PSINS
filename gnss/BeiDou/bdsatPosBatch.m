function [satPos, satClkCorr, TGD, orbitp] = bdsatPosBatch(transmitTime, eph)
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/07/2015
global gbd
    s = gbd.ephs;
    
    dt = transmitTime - eph(:,s.Toc);
    idx=dt>302400; dt(idx)=dt(idx)-604800; idx=dt<-302400; dt(idx)=dt(idx)+604800; 
    satClkCorr = eph(:,s.af0) + eph(:,s.af1).*dt+eph(:,s.af2).*dt.*dt;   % 钟差改正（0.1ms量级）
    
    tk = transmitTime - satClkCorr - eph(:,s.Toe);                    % 归化时间
    idx=dt>302400; tk(idx)=tk(idx)-604800; idx=tk<-302400; tk(idx)=tk(idx)+604800; 

    a   = eph(:,s.sqrtA).^2;      % 长半轴
    n0  = sqrt(gbd.GM./a.^3);    % 平均角速率
    n   = n0 + eph(:,s.Deltan);   % 摄动改正

    M   = eph(:,s.M0) + n.*tk;     % 平近点角
    E   = M;  dE  = 1.0;          % 偏近点角
    while max(abs(dE)) > 1.e-12
        E_old   = E;
        sinE    = sin(E);
        E       = M + eph(:,s.e).*sinE;
        dE      = E-E_old;
    end
    cosE = cos(E);
    nu   = atan2(sqrt(1-eph(:,s.e).^2).*sinE, cosE-eph(:,s.e));  % 真近点角
    
    phi  = nu + eph(:,s.omega);   % 升交角距
    sin2phi = sin(2*phi); cos2phi = cos(2*phi);

    u = phi +                             eph(:,s.Cuc).*cos2phi + eph(:,s.Cus).*sin2phi;  % 升交角距改正
    r = a.*(1-eph(:,s.e).*cosE) +         eph(:,s.Crc).*cos2phi + eph(:,s.Crs).*sin2phi;  % 失径及改正
    i = eph(:,s.i0) + eph(:,s.iDot).*tk + eph(:,s.Cic).*cos2phi + eph(:,s.Cis).*sin2phi;  % 轨道倾角及改正

    sinu = sin(u); cosu = cos(u);
    sini = sin(i); cosi = cos(i);
    x = r.*cosu; y = r.*sinu;  % 卫星在轨道平面中坐标（注意：X轴指向升交点，而不是近地点）
    % satellite is GEO
    isGEO = eph(:,s.PRN)<=505 & eph(:,s.PRN)>=500;
    if ~isempty(tk(isGEO))
        Omega = eph(isGEO,s.OMEGA0) + eph(isGEO,s.OMEGADot).*tk(isGEO) - gbd.wie*eph(isGEO,s.Toe);
        sinOmega = sin(Omega); cosOmega = cos(Omega);
        xyz = [ x(isGEO).*cosOmega-y(isGEO).*cosi(isGEO).*sinOmega, x(isGEO).*sinOmega+y(isGEO).*cosi(isGEO).*cosOmega, y(isGEO).*sini(isGEO) ];
        wtk = gbd.wie*tk(isGEO);
        swtk = sin(wtk);      cwtk = cos(wtk);
        s5 = sin(-5*pi/180);  c5 = cos(-5*pi/180);
        satPos = zeros(length(tk),3);
        satPos(isGEO,1) =  cwtk.*xyz(:,1) + swtk.*c5.*xyz(:,2) + swtk.*s5.*xyz(:,3);
        satPos(isGEO,2) = -swtk.*xyz(:,1) + cwtk.*c5.*xyz(:,2) + cwtk.*s5.*xyz(:,3);
        satPos(isGEO,3) =               0 -       s5.*xyz(:,2) +       c5.*xyz(:,3);
    end
    % satellite is IGSO/MEO
    noGEO = ~isGEO;
    if ~isempty(tk(noGEO))
        Omega = eph(noGEO,s.OMEGA0) + (eph(noGEO,s.OMEGADot)-gbd.wie).*tk(noGEO) - gbd.wie*eph(noGEO,s.Toe); % 升交点经度
        sinOmega = sin(Omega); cosOmega = cos(Omega);
        satPos(noGEO,:) = [ x(noGEO).*cosOmega-y(noGEO).*cosi(noGEO).*sinOmega, x(noGEO).*sinOmega+y(noGEO).*cosi(noGEO).*cosOmega, y(noGEO).*sini(noGEO) ];  % ECEF坐标
    end
    
    satClkCorr(:,2) = satClkCorr + gbd.F*eph(:,s.e).*eph(:,s.sqrtA).*sinE;   % 钟差相对论改正（10ns量级，对卫星位置影响小）
    TGD = [eph(:,s.TGD1), eph(:,s.TGD2), (eph(:,s.TGD2)-gbd.kf*eph(:,s.TGD1))/(1-gbd.kf)];

	orbitp = [dt, tk, i, nu, u, r];
