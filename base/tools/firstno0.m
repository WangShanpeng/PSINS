function [res, idx] = firstno0(scr, clm)
% To get the first non-zero data for specific columns.
%
% Prototype: [res, k1] = firstno0(scr, clm)
% Inputs: scr - data source input
%         clm - column for non-zero
% Outputs: res - result
%          idx - row index for the first non-zero data
%
% See also  no0, norep.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 20/04/2021
    if nargin<2, clm=1:size(scr,2)-1; end
    nm = normv(scr(:,clm));
    idx = find(nm>0,1,'first');
    res = scr(idx,clm);

