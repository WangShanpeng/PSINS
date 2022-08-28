function [res, idx] = no0(scr, clm)
% To get non-zero data for specific columns.
%
% Prototype: res = no0(scr, clm)
% Inputs: scr - data source input
%         clm - column for non-zero
% Outputs: res - result
%          idx - row index for non-zero data
%
% See also  firstno0, nonan, norep, norep0, normv.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/08/2020
    if nargin==2,   nv = normv(scr(:,clm));
    else,           nv = normv(scr(:,1:end-1)); end
    res = scr(nv>0, :);
    if nargout>1
        idx = (1:length(scr))';
        idx = idx(nv>0);
    end
