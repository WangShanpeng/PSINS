function [hms, hms1] = s2hms(s)
% Convert time unit from second to hour+minute+second
%
% Prototype: [hms,hms1] = s2hms(s)
% Input: s - seconds
% Outputs: hms - hh:hour, mm:min, ss:sec.
%          hms1 - hms in array.
% Example:
%    [hms,hms1] = s2hms(123456.78);     
%       >>  hms = 34 17 36.78
%
% See also  hms2s, dms2r.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/11/2021
    hh = fix(s/3600.0);
    min = fix((s-hh*3600.0)/60.0); 
    sec = s-hh*3600.0-min*60.0;
	hms = hh*10000+min*100+sec;
    hms1 = [hh,min,sec];

