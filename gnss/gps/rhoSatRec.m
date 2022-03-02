function [rho, LOS, AzEl, BLH, Cen] = rhoSatRec(satPos, recPos, rho0)
% Calculate distances between satellites and receiver, taking into
% account the Earth rotation.
%
% Prototype: [rho, LOS] = rhoSatRec(satPos, recPos, rho0)
% Inputs: satPos - satellites positions in ECEF at transmission time
%         recPos - receiver position in ECEF at reception time
%         rho0 - initial distances between satellites and receiver
% Outputs: rho - distances between satellites and receiver
%          LOS - satellite line-of-sight vectors in ECEF at reception time
%          AzEl - satellite azimuth(s) and elevation(s)
%
% See also  satPosVel, lsPos, topocent.

% Originated by Yangling 2008/3/25, Tongji Unv.,China
% Modified by Yangongmin 2013/9/16, NWPU
global ggps
    recPos = repmat(recPos(1:3)',size(satPos,1),1);
	if nargin<3, dpos=satPos-recPos; rho0=sqrt(dpos(:,1).^2+dpos(:,2).^2+dpos(:,3).^2); end
    wtau = ggps.wie*rho0/ggps.c;
    sw = sin(wtau);  cw = cos(wtau);
    satRot = [cw.*satPos(:,1)+sw.*satPos(:,2), -sw.*satPos(:,1)+cw.*satPos(:,2), satPos(:,3)];
    dpos = satRot - recPos;
    rho = sqrt(dpos(:,1).^2+dpos(:,2).^2+dpos(:,3).^2);
    LOS = dpos./[rho,rho,rho];
    % topocent
    [BLH, Cen] = xyz2blh(recPos(1,1:3));
    Ln = LOS*Cen;  % Ln: LOS in receiver n-frame
    AzEl = [atan2(Ln(:,1), Ln(:,2)), atan(Ln(:,3)./sqrt(Ln(:,1).^2+Ln(:,2).^2))];
    idx = find(AzEl(:,1)<0);
    AzEl(idx,1) = AzEl(idx,1) + 2*pi;
