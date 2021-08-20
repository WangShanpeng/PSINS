function templot(temp, tm)
% Temperature plot.
%
% Prototype: templot(temp)
% Input: temp - temperature
%
% See also  od, imuplot, gpsplot, magplot.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/06/2021
    if nargin<2, tm='t/s'; end
    myfig;
    tscalepush(tm);
    plot(temp(:,end)/tscaleget(), temp(:,1:end-1)); xygo('Temp');
    tscalepop();