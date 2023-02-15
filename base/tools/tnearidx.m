function idx = tnearidx(t, ti)
% Get the nearest time index, ti within t.
%
% Prototype: idx = tnearidx(t, ti)
%
% See also  combinet, adddt.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 18/12/2022
    n = length(ti);
    [~, idx] = sort([ti-1.0e-6; t]);
    idx = find(idx<=n) - (0:n-1)';