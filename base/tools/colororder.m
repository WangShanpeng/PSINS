function ck = colororder(k)
% Get the k-th colororder in the current axis.
%
% Prototype: ck = colororder(k)
%
% See also  xxx.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 18/05/2022
%     c = get(gca,'ColorOrder');
    c = [
        0                   0                   1.000000000000000
        0   0.500000000000000                   0
        1.000000000000000                   0                   0
        0   0.750000000000000   0.750000000000000
        0.750000000000000                   0   0.750000000000000
        0.750000000000000   0.750000000000000                   0
        0.250000000000000   0.250000000000000   0.250000000000000
        0    0.75  0.75
        0.5  0.5   0.5
        0    1     0
    ];
    ck = c(mod(k-1,size(c,1))+1,:);