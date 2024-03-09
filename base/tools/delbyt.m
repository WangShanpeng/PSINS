function res = delbyt(x1, x2)
% Delete by time tag.
%
% Prototype:  res = delbyt(x1, x2)
% Inputs: x1,x2 - input data
% Output: res = x1-x2
%
% See also  delbias, deltrend.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/04/2023 
    [t, idx1, idx2] = intersect(x1(:,end), x2(:,end));
    res = [x1(idx1,1:end-1)-x2(idx2,1:end-1), t];