function pos = dratt(att, od, pos0, inst, kod)
% Dead Reckoning(DR) using attitude array.
%
% Prototype: pos = dratt(att, od, pos0, inst, kod)
% Inputs: att -attitude array
%         od - odometer distance increment array
%         pos0 - initial position
%         inst - ints=[dpitch;0;dyaw], where dpitch and dyaw are
%            installation error angles(in rad) from odometer to SIMU
%         kod - odometer scale factor
% Output: pos - DR position
%
% See also  drupdate, drpure, drinit, drcalibrate.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/01/2021
    if nargin<5, kod=1; end
    if nargin<4, inst=[0;0;0]; end
    if nargin<3, pos0=[0;0;0]; end
    attod = combinedata(att(:,[1:3,end]), od);
    Cnb = a2matBatch(attod(:,1:4));
    Cbo = a2mat(-inst)*kod;
	prj = Cbo*[0;1;0];
    dSb = [prj(1)*attod(:,5), prj(2)*attod(:,5), prj(3)*attod(:,5)];
    dSn = [ Cnb(:,1).*dSb(:,1)+Cnb(:,2).*dSb(:,2)+Cnb(:,3).*dSb(:,3), ...
            Cnb(:,4).*dSb(:,1)+Cnb(:,5).*dSb(:,2)+Cnb(:,6).*dSb(:,3), ...
            Cnb(:,7).*dSb(:,1)+Cnb(:,8).*dSb(:,2)+Cnb(:,9).*dSb(:,3) ];
    pos0 = repmat(pos0',length(attod),1);  % dxyz2pos
    pos = pos0;
    for k=1:3  % iteration for calcuating DR pos, faster than element for-loop 
        [RMh, clRNh] = RMRN(pos);
        dpos = [dSn(:,2)./RMh, dSn(:,1)./clRNh, dSn(:,3)];
        dpos = cumsum(dpos);
        pos = pos0 + dpos;
    end
    pos(:,4) = attod(:,4);
