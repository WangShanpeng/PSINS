function [avp, vpgps, lvgps, vpod, lvod, kappa, dt, dyaw, dkgzz] = igoload(fname, t0)
% SINS/GNSS/OD output bin file load for C++ class 'CSINSGNSSOD':
% void CSINSGNSSOD::operator<<(CFileRdWt &f)
% {
% 	f<<sins.att<<sins.vn<<sins.pos<<sins.eb<<sins.db  // 1-15
% 		<<sins.vnL<<sins.posL<<lvGNSS   // 16-24
% 		<<vnOD<<posOD<<lvOD<<ODKappa()  // 25-36
% 		<<dtGNSSdelay<<dyawGNSS<<(sins.Kg.e22-1.0) <<kftk; // 37-40
% }
%
% Prototype: [avp, vpgps, lvgps, vpod, lvod, kappa, dt, dyaw, dkgzz] = igoload(fname, t0)
% Inputs: fname - SINS/GNSS/OD bin file name
%         t0 - start time
% Outputs: xxx
%          
% See also  igoplot, igload.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/01/2023
    igo = binfile(fname, 40);
    if nargin>1, igo(:,end) = igo(:,end)-t0; end
	avp = igo(:,[1:15,end]);
    vpgps = igo(:,[16:21,end]);  vpod = igo(:,[25:30,end]);
    lvgps = igo(:,[22:24,end]);  lvod = igo(:,[31:33,end]); 
    kappa = igo(:,[34:36,end]);
    dt = igo(:,[37,end]);  dyaw = igo(:,[38,end]);  dkgzz = igo(:,[39,end]);

