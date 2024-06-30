function dr = drinit(avp0, inst, kod, ts, Td)
% Dead Reckoning(DR) structure array initialization.
%
% Prototype: dr = drinit(avp0, inst, ts)
% Inputs: avp0 - initial avp0 = [att0; vn0; pos0], vn0 may omitted
%         inst - ints=[dpitch;aos;dyaw], where dpitch and dyaw are
%            installation error angles(in rad) from odometer to SIMU
%            aos is Angle Of Slide coefficient
%         kod - odometer scale factor in meter/pulse.
%         ts - SIMU % odometer sampling interval
%         Td - leveling time constant
% Output: dr - DR structure array
%
% See also  drupdate, drpure, drcalibrate, insinit.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/12/2008, 8/04/2014, 08/04/2023
	dr = [];
    avp0 = avp0(:);
    if length(avp0)<9, avp0 = [avp0(1:3); zeros(3,1); avp0(4:end)]; end
	dr.qnb = a2qua(avp0(1:3)); dr.vn = zeros(3,1); dr.pos = avp0(7:9);
    [dr.qnb, dr.att, dr.Cnb] = attsyn(dr.qnb);
    dr.avp = [dr.att; dr.vn; dr.pos];
	dr.kod = kod;
    dr.aos = inst(2); inst(2)=0;  % AOS - angle of silde coefficient
    dr.Cbo = a2mat(-inst)*kod;
	dr.prj = dr.Cbo*[0;1;0]; % from OD to SIMU
	dr.ts = ts;
	dr.distance = 0;  dr.distance1 = 0;
	dr.eth = earth(dr.pos); dr.web = [0;0;0];
    dr.Mpv = [0, 1/dr.eth.RMh, 0; 1/dr.eth.clRNh, 0, 0; 0, 0, 1];
    if nargin<5, Td=0; end
    dr.Td = Td;
    if Td>0  % for leveling
        xi=0.707; xi2=xi*xi; ws2=9.8/6738160; sigma=2*pi*xi/(Td*sqrt(1.0-xi2)); sigma2=sigma*sigma;
        dr.gck(1) = 3.0*sigma; 
        dr.gck(2) = sigma2*(2.0+1.0/xi2)/ws2-1.0; 
        dr.gck(3) = sigma2*sigma/(9.8*xi2);
        dr.wnc = zeros(3,1); dr.vni = zeros(3,1); dr.dpos = zeros(3,1);
    end
