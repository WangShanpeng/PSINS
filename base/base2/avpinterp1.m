function avp = avpinterp1(avp, t, method)
% avp linear interpolation. 
%
% Prototype: avp = avpinterp1(avp, t, method)
% Inputs: avp - input avp
%         t - time tag, or sampling interval ts
%         method - 'nearest'/'linear' etc, see interp1
% Output: avp - interpolated avp
%
% See also  attinterp, attinterp1.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 13/11/2019
    if ~exist('method', 'var'), method = 'linear'; end
    if length(t)==1; t = (avp(1,end):t:avp(end,end))'; end
    i1 = find(t>=avp(1,end),1,'first');
    i2 = find(t<=avp(end,end),1,'last');
    t = t(i1:i2);
    avp(:,1:3) = att2c(avp(:,1:3));
    avp = [interp1(avp(:,end), avp(:,1:end-1), t, method), t];
    avp(:,1:3) = iatt2c(avp(:,1:3));
