function idx = zerotest(val, dist, h)
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
    if nargin<3, h=1e-10; end
    if nargin<2, dist=10; end
    val = abs(val);
    zr = (val<h);
    [~, idx] = norep(zr, 1, 0);
    idx = (find(zr==1 & idx==1));
    idx = idx(diff(idx)>=dist);
    idx = idx+dist;
    idx = idx(val(idx)<h);