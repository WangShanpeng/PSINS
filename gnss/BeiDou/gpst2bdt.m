function [bdsow, bdwn, bdt] = gpst2bdt(gpssow, gpswn)
% Convert GPST week/time to BDT week/time.
%
% Prototype: [bdsow, bdwn, bdt] = gpst2bdt(gpssow, gpswn)
% Inputs: gpssow - second of week for GPS
%         gpswn - week number for GPS
% Outputs: bdsow - second of week for BD
%         bdwn - week number for BD
%         bdt - BDT = bdwn*604800 + bdtow
%
% NOTES: UTC[2006/1/1,0:0:0]=GPST[2006/1/1,0:0:0]-14
%        GPST[2006/1/1,0:0:0]=820108800
%  then, BDT = GPST - 820108800 - 14
%
% See also  cal2gpst, cal2bdt.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/07/2015
    if nargin==2  % [bdsow, bdwn, bdt] = gpst2bdt(gpssow, gpswn)
        gpst = gpswn*604800 + gpssow;
    elseif nargin==1 && gpssow<=604800  %  bdsow = gpst2bdt(gpssow) 
        bdsow = mod(gpssow-14,604800);
        return;
    else % [bdsow, bdwn, bdt] = gpst2bdt(gpst)
        gpst = gpssow;
    end
    bdt = gpst - 820108800 -14;
    bdwn = fix(bdt/604800);
    bdsow = bdt - bdwn*604800;

