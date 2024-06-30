function dr = drupdate(dr, imu, dS)
% Dead Reckoning(DR) attitude and position updating.
%
% Prototype: dr = drupdate(dr, imu, dS)
% Inputs: dr - DR structure array, created by 'drinit'
%         imu - SIMU gyro/acc sampling data
%         dS - odometer distance increment
% Output: dr - DR structure array after DR updating
%
% See also  drinit, drpure, dratt, drcalibrate, nhcpure, insupdate.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/12/2008, 8/04/2014
    nts = dr.ts*size(imu,1);
    dr.distance = dr.distance + dr.kod*norm(dS);
    dr.distance1 = dr.distance1 + dr.kod*dS;
    [phim, dvbm] = cnscl(imu);  qnb12 = qupdt(dr.qnb, phim/2);
    if length(dS)>1,
        dSn = qmulv(qnb12, dr.Cbo*dS);
    else
        dSn = qmulv(qnb12, dr.prj*dS);
    end
%     dSn = rotv([0;0;-dr.aos*norm(dS)/nts*phim(3)/nts], dSn);
    dSn = rotv([0;0;-dr.aos*phim(3)/nts], dSn);
    dr.vn = dSn/nts;
    dr.eth = earth(dr.pos, dr.vn);
    dr.web = phim/nts-dr.Cnb'*dr.eth.wnie;
    dr.Mpv = [0, 1/dr.eth.RMh, 0; 1/dr.eth.clRNh, 0, 0; 0, 0, 1];
    dr.pos = dr.pos + dr.Mpv*dSn; % see my PhD thesis Eq.(4.1.1)
    dr.qnb = qupdt(dr.qnb, phim-dr.Cnb'*dr.eth.wnin*nts);
    [dr.qnb, dr.att, dr.Cnb] = attsyn(dr.qnb);
    dr.avp = [dr.att; dr.vn; dr.pos];
    if dr.Td>0.01  % leveling
		fn=qmulv(dr.qnb, dvbm/nts);
        dVE = dr.vni(1) - dr.vn(1);     % vni: inertial vel;  vn: ref vel
        dr.vni(1) = dr.vni(1) + (fn(1)-dr.gck(1)*dVE)*nts;
        dr.dpos(1) = dr.dpos(1) + dVE*dr.gck(3)*nts;
        dr.wnc(2) = dVE*(1+dr.gck(2))/6378137 + dr.dpos(1);
        dVN = dr.vni(2) - dr.vn(2);
        dr.vni(2) = dr.vni(2) + (fn(2)-dr.gck(1)*dVN)*nts;
        dr.dpos(2) = dr.dpos(2) + dVN*dr.gck(3)*nts;
        dr.wnc(1) = -dVN*(1+dr.gck(2))/6378137 - dr.dpos(2);
        dr.qnb = qmul(rv2q(-dr.wnc*nts),dr.qnb);
        dr.avp = [dr.att; dr.vni; dr.pos];
    end