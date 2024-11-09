function [m, ij] = max2(scr)
% max element of 2D matrix.
%
% Prototype: [m, ij] = max2(scr)
% Input: scr - 2D data source input to find max
% Output: m - max element
%         ij - i-row,j-column
%
% See also  maxn.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/09/2024
    [m, i] = max(scr);
    [m, j] = max(m);
    ij = [i(j);j];