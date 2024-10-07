function vp = vpzupt1(vp)
% Zero data test.
%
% Prototype: idx = zerotest(val, dist, h)
% Inputs: val - column data to test
%         h - zero threshhold
%         dist - data interval to hold zeros
% Output: idx - zero data index
% 
% See also  accstatic, imustatic, no0, norep.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/06/2024
    if size(vp,2)>7, vp=vp(:,[4:9,end]); end
    t = (vp(:,end)-vp(1,end))/(vp(end,end)-vp(1,end));  dt = diff([vp(1,end); vp(:,end)]);
    verr = [t*vp(end,1), t*vp(end,2), t*vp(end,3)];
    [RMh, clRNh] = RMRN(vp(1,4:6)');
    perr = cumsum([verr(:,2).*dt/RMh, verr(:,1).*dt/clRNh, verr(:,3).*dt]);
    vp = [vp(:,1:3)-verr, vp(:,4:6)-perr, vp(:,end)];
