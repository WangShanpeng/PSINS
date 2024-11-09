function nd = gnumdop(numdop, maxdop)
% Trans GNSS satNum&DOP to compressed array, or reverse.
% 
% Prototype: nd = gnumdop(numdop, maxdop)
% Inputs: numdop - [satNum£¬DOP] 2-column array, or 1-column array
%         max dop - max DOP limitation
% Output: nd = satNum+DOP/1000, or 2-column array
%
% See also  gpsplot.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/09/2024
    if nargin<2, maxdop=9.99; end
    if size(numdop,2)==1   %  satNum.dop -> [satNum, dop]
        satnum = fix(numdop); nd = [satnum, (numdop-satnum)*1000];
    else                  %   [satNum, dop] -> satNum.dop 
        numdop(numdop(:,2)>maxdop,2) = maxdop;
        nd = numdop(:,1) + numdop(:,2)/1000;
    end
