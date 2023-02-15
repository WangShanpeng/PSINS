function [avp, vpgps, lvgps, dt, dyaw] = igload(fname, t0)
% SINS/GNSS output bin file load for C++ class 'CSINSGNSS':
% void CSINSGNSS::operator<<(CFileRdWt &f)
% {
% 	f<<sins.att<<sins.vn<<sins.pos<<sins.eb<<sins.db  // 1-15
% 		<<sins.vnL<<sins.posL<<lvGNSS   // 16-24
% 		<<dtGNSSdelay<<dyawGNSS <<kftk; // 25-27
% }
%
% Prototype: [avp, vpgps, lvgps, dt, dyaw] = igload(fname, t0)
% Inputs: fname - SINS/GNSS/OD bin file name
%         t0 - start time
% Outputs: xxx
%          
% See also  igplot, igoload, insload.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/01/2023
    ig = binfile(fname, 40);
    if nargin>1, ig(:,end) = ig(:,end)-t0; end
	avp = ig(:,[1:15,end]);
    vpgps = ig(:,[16:21,end]);
    lvgps = ig(:,[22:24,end]);
    dt = ig(:,[25,end]);  dyaw = ig(:,[26,end]);

