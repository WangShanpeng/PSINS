function res = remn(x, n)
% Remainder, res = x - fix(x./n).*n.
%
% Prototype: res = remn(x, n)
% Inputs: x, n - data source input to be averaged
% Output: res - = [1,2,...,n] ( not [1,2,...,0] )
%
% See also  rem.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/07/2021
    res = rem(x, n);
    if res==0, res=n; end

