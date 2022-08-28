function [res, idx] = nonan(scr, clm)
% To get non-NAN data for specific columns.
%
% Prototype: res = nonan(scr, clm)
% Inputs: scr - data source input
%         clm - column for non-zero
% Outputs: res - result
%          idx - row index for non-NAN data
%
% See also  no0, firstno0, norep, normv.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/06/2021
    if nargin==2,   nv = normv(scr(:,clm));
    else,           nv = normv(scr(:,1:end-1)); end
    res = scr(~isnan(nv), :);
    if nargout>1
        idx = (1:length(scr))';
        idx = idx(~isnan(nv));
    end
