function [sow, wn, gpst] = cal2gpst(ymdhms)
% Convert GPST from calendar day/time to week/time.
%
% Prototype: [sow, wn, gpst] = cal2gpst(ymdhms)
% Input: ymdhms - calendar day/time array: [year, month, day, hour, minute, second]
% Outputs: sow - second of week
%          wn - week number
%          gpst - GPST = wn*604800 + tow
%
% See also  rinexReadN, rinexReadO.

% Originated by Yangling 2008/3/25, Tongji Unv.,China
% Modified by Yangongmin,  30/09/2013, 21/07/2015
    if length(ymdhms)<6,  ymdhms(6) = 0;  end
    year = ymdhms(1); month = ymdhms(2);  day = ymdhms(3);
    hour = ymdhms(4); minute = ymdhms(5); second = ymdhms(6);
    if year<=79,     year = year+2000;
    elseif year<=99, year = year+1900;    end
    if month<=2
        year = year-1;   month = month+12;
    end
%     UT = hour+minute/60.0+second/3600.0;
%     JD = fix(365.25*year)+fix(30.6001*(month+1))+day+UT/24.0+1720981.5;
%     gpst = (JD-2444244.5)*24*3600;  % CAL[1980/1/6,0:0:0]==JD[2444244.5]
    JD = fix(365.25*year)+fix(30.6001*(month+1))+day+1720981.5;
    gpst = (JD-2444244.5)*24*3600 + (hour*3600+minute*60+second);
    wn = fix(gpst/604800);  % 604800=3600*24*7;
    sow = (gpst*1000-wn*604800*1000)/1000;  

