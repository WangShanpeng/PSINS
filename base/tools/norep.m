function [res, idx] = norep(scr, clm, delrep)
% To get non-repeat data for specific columns.
%
% Prototype: [res, idx] = norep(scr, clm, delrep)
% Inputs: scr - data source input
%         clm - column for non-repeat
%         delrep - =1 for delete repeated row; =0 for set repeated row as 0.
% Outputs: res - result
%          idx - non-repeat index
%
% See also  no0, norep0, setrep0, normv, scaleclm.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/09/2020
    if nargin<3, delrep=1; end
    if nargin<2, clm=1:size(scr,2)-1; end
    nv = normv(scr(:,clm));
    nv = diff([nv(1)-1;nv]);
    idx = abs(nv)>0;
    if delrep==1, res = scr(idx, :);
    else          res = scr; res(~idx,clm) = 0; end
