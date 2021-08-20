function [sow, wn, bdt] = cal2bdt(ymdhms)
% Convert BDT from calendar day/time to week/time.
%
% Prototype: [sow, wn, gpst] = cal2bdt(ymdhms)
% Input: ymdhms - calendar day/time array: [year, month, day, hour, minute, second]
% Outputs: sow - second of week
%          wn - week number
%          bdt - BDT = wn*604800 + tow
%
% See also  cal2gpst, gpst2bdt.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/07/2015
    [sow, wn, gpst] = cal2gpst(ymdhms);
    bdt = gpst - 820108800;   % GPST[2006/1/1,0:0:0]=820108800
    wn = fix(bdt/604800);
    sow = bdt - wn*604800;
    
