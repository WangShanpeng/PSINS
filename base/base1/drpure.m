function avp = drpure(imuod, avp0, inst, kod)
% Dead Reckoning(DR) attitude and position updating.
%
% Prototype: avp = drpure(imuod, avp0, inst, kod)
% Inputs: imuod = [gx, gy, gz, dS, t] where gx/gy/gz for gyro angular increment,
%                 dS for OD distance increment, t for time tag
%         avp0 - initial [att0, vn0, pos0]
%         inst - ints=[dpitch;0;dyaw], where dpitch and dyaw are
%            installation error angles(in rad) from odometer to SIMU
%         kod - odometer scale factor in meter/pulse.
% Output: avp - [att, vn, pos] output array
%
% See also  dratt, drinit, drupdate, inspure.

% Copyright(c) 2009-2019, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 9/12/2019
    if nargin<4, kod = 1; end
    if nargin<3, inst = [0;0;0]; end
    [nn, ts] = nnts(2, diff(imuod(1:2,end)));
    if length(avp0)<9, avp0=[avp0(1:3);zeros(3,1);avp0(4:end)]; end
    dr = drinit(avp0, inst, kod, ts);
    len = size(imuod,1);
    avp = zeros(fix(len/nn), 10);
    ki = timebar(nn, len, 'Pure DR navigation processing.');
    for k=1:nn:len-nn+1
        k1 = k+nn-1;
        dr = drupdate(dr, imuod(k:k1,1:3), sum(imuod(k:k1,end-1)));
        avp(ki,:)  = [dr.avp; imuod(k1,end)]';
        ki = timebar;
    end
    insplot(avp);
