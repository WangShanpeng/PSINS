function [md, d] = MJD(year, month, day)
% ModJDate calculation
%
% Examples:
%   MJD(1858,11,17), % = 0
%   MJD(2000,1,15), % = 51558
%
% See also  JD, GMST, GAST.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/11/2021
    if nargin==1
        day = year(3); month = year(2); year = year(1);
    end
    d = JD(year, month, day);
    md = d - 2400000.5;