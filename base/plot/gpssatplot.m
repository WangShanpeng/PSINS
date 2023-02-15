function gpssatplot(satnumdop, s)
% GPS sattlite number & DOP plot.
%
% Prototype: gpssatplot(satnumdop)
% Inputs: satnumdop - sattlite number & DOP data, satnumdop=satnum+dop/s
%         s - DOP scale
%          
% See also  gpsplot.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/12/2022
    if nargin<2, s=1000; end
    [m, n] = size(satnumdop);
    if n==4 || n==7, error('No SatNum & DOP information found!'); end
    if n==1, t=(1:m)'; satnumdop=[satnumdop,t]; else, t = satnumdop(:,end); end
    satnum = fix(satnumdop(:,end-1));
    dop = (satnumdop(:,end-1)-satnum)*s;  sdop=sum(dop);
    myfig;
    if sdop<1  % no dop info
        plot(t, satnum); xygo('SatNum');
    else
        subplot(211), plot(t, satnum); xygo('SatNum');
        subplot(212), plot(t, dop); xygo('DOP');
%         plot(t, satnum, t, dop*10); xygo('SatNum, DOP');  lengend('SatNum', 'DOP x10');
    end
    