function eth = attachdgn(eth, dgn)
% Update the Earth related parameters, much faster than 'earth'.
%
% Prototype: eth = ethupdate(eth, pos, vn)
% Inputs: eth - input earth structure array
%         dgn - Gravity abnomal & Deflection of vertical, [dgE;dgN;dgU,t]
% Output: eth - parameter structure array
%
% See also  ethupdate.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 13/08/2022
    eth.dgn = dgn;  eth.dgnlen = length(dgn);
    eth.dgnk = 1;
    eth.dgnt = dgn(eth.dgnk,end);

