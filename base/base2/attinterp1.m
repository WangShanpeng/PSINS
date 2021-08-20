function att = attinterp1(att, t, method)
% att linear interpolation. 
%
% Prototype: att = attinterp1(att, t, method)
% Inputs: att - input attitude
%         t - time tag
%         method - 'nearest'/'linear' etc, see interp1
% Output: att - interpolated attitude
%
% See also  avpinterp1, att2c, interp1.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 13/11/2019
    if ~exist('method', 'var'), method = 'linear'; end
    i1 = find(t>att(1,end),1,'first');
    i2 = find(t<att(end,end),1,'last');
    t = t(i1:i2);
    att(:,1:3) = att2c(att(:,1:3));
    att = [interp1(att(:,end), att(:,1:end-1), t, method), t];
    att(:,1:3) = iatt2c(att(:,1:3));