function d = JD(year, month, day)
% JulianDate calculation, should yymmdd>1582.10.15
% Ref. https://blog.csdn.net/zhuimengshizhe87/article/details/26702997
% https://blog.csdn.net/qq_24172609/article/details/112244135?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522162115452716780357279082%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=162115452716780357279082&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_v2~rank_v29-3-112244135.first_rank_v2_pc_rank_v29&utm_term=%E5%85%AC%E5%85%83%E7%BA%AA%E5%B9%B4%E6%B3%95&spm=1018.2226.3001.4187
%
% Examples:
%   JD(1858,11,17), % = 2400000.5
%   JD(1992,2,1), % = 2448653.5
%
% See also  MJD, GMST, GAST.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/11/2021
    if nargin==1
        day = year(3); month = year(2); year = year(1);
    end
    if month>2, month=month+1; 
    else, year=year-1; month=month+13; end
    d = fix(365.25*(year+4716))+fix(30.6001*month)+day - 1524.5;
    ja = fix(0.01*year);
    d = d + (2-ja+fix(0.25*ja));
    