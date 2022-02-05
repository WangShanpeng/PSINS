function s = hms2s(hhmmss)
% Convert time unit from hour+minute+second to second
%
% Prototype: s = hms2s(hhmmss)
% Input: hhmmss - hh:hour, mm:min, ss:sec.
% Output: s - seconds
% Example:
%    s = hms2s(123456.78);  % or  s = hms2s([12 ,34, 56.78]);      
%       >>  s = 45296.78
%
% See also  s2hms, dms2r.

% Copyright(c) 2009-2018, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/05/2018
    if size(hhmmss,2)==3, hhmmss=hhmmss(:,1)*10000+hhmmss(:,2)*100+hhmmss(:,3); end
    hh = fix(hhmmss/10000.0);
    min = fix((hhmmss-hh*10000.0)/100.0); 
    sec = hhmmss-hh*10000.0-min*100.0;
	s = hh*3600+min*60+sec;

