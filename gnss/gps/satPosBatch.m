function [satPos, satClkCorr, TGD, orbitp] = satPosBatch(transmitTime, eph)
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/08/2013, 30/06/2015
global ggps
    s = ggps.ephs;
    
    dt = transmitTime - eph(:,s.Toc);
    idx=dt>302400; dt(idx)=dt(idx)-604800; idx=dt<-302400; dt(idx)=dt(idx)+604800; 
    satClkCorr = eph(:,s.af0) + eph(:,s.af1).*dt+eph(:,s.af2).*dt.*dt;   % 钟差改正（0.1ms量级）
    
    tk = transmitTime - satClkCorr - eph(:,s.Toe);                    % 归化时间
    idx=dt>302400; tk(idx)=tk(idx)-604800; idx=tk<-302400; tk(idx)=tk(idx)+604800; 

    a   = eph(:,s.sqrtA).^2;      % 长半轴
    n0  = sqrt(ggps.GM./a.^3);    % 平均角速率
    n   = n0 + eph(:,s.Deltan);   % 摄动改正

    M   = eph(:,s.M0) + n.*tk;     % 平近点角
    E   = M;  dE  = 1.0;          % 偏近点角
    while max(abs(dE)) > 1.e-12
        E_old   = E;
        sinE    = sin(E);
        E       = M + eph(:,s.e).*sinE;
        dE      = E-E_old;
    end
    if max(abs(E-eph(:,s.e).*sin(E)-M))>1e-10, error('E error'); end
    cosE = cos(E);
    nu   = atan2(sqrt(1-eph(:,s.e).^2).*sinE, cosE-eph(:,s.e));  % 真近点角
    
    phi  = nu + eph(:,s.omega);   % 升交角距
    sin2phi = sin(2*phi); cos2phi = cos(2*phi);

    u = phi +                             eph(:,s.Cuc).*cos2phi + eph(:,s.Cus).*sin2phi;  % 升交角距改正
    r = a.*(1-eph(:,s.e).*cosE) +         eph(:,s.Crc).*cos2phi + eph(:,s.Crs).*sin2phi;  % 失径及改正
    i = eph(:,s.i0) + eph(:,s.iDot).*tk + eph(:,s.Cic).*cos2phi + eph(:,s.Cis).*sin2phi;  % 轨道倾角及改正

    Omega = eph(:,s.OMEGA0) + (eph(:,s.OMEGADot)-ggps.wie).*tk - ggps.wie*eph(:,s.Toe); % 升交点经度
    
    sinu = sin(u); cosu = cos(u);
    sini = sin(i); cosi = cos(i);
    sinOmega = sin(Omega); cosOmega = cos(Omega);
    x = r.*cosu; y = r.*sinu;  % 卫星在轨道平面中坐标（注意：X轴指向升交点，而不是近地点）
    satPos = [ x.*cosOmega-y.*cosi.*sinOmega, x.*sinOmega+y.*cosi.*cosOmega, y.*sini ];  % ECEF坐标
    
    satClkCorr(:,2) = satClkCorr + ggps.F*eph(:,s.e).*eph(:,s.sqrtA).*sinE;   % 钟差相对论改正（10ns量级，对卫星位置影响小）
    TGD = eph(:,s.TGD);
    
    orbitp = [dt, tk, i, Omega, nu, u, r];
            