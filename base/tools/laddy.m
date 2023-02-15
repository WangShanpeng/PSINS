function laddy(y, isabs)
% Selected line add y-value.
%
% Prototype: laddy(y, isabs)
% Inputs: y - y to be addedd
%         isabs - is absolute x,y flag
% Output: N/A
% 
% See also ladd, lneg, lmul, lmc, avplmc, tshift, scft0.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/10/2022
    if nargin<2, isabs=0; end
    ladd(0,y,isabs);