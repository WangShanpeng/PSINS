function [res, idx] = norep0(scr, clm)
% To get non-repeat & non-zero data for specific columns.
%
% Prototype: [res, idx] = norep0(scr, clm)
% Inputs: scr - data source input
%         clm - column for non-repeat & non-zero
% Outputs: res - result
%          idx - non-repeat index
%
% See also  no0, norep, setrep0.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/08/2022
    if nargin<2, clm=1:size(scr,2)-1; end
    [scr, ~] = norep(scr, clm, 1);
    [res, idx] = no0(scr, clm);
