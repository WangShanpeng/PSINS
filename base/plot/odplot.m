function [sv, sumod] = odplot(od, nsmooth, isplot)
% Odometer plot.
%
% Prototype: sv = odplot(od, nsmooth, isplot)
% Inputs: od - [od_increment, t].
%         nsmooth = smooth points
%         isplot - plot flag
% Output: sv - smooth velocity
%
% See also  odanalysis, dvlplot, odpplot, distplot, imuplot, imuodplot, gpsplot, magplot, avpfile.

% Copyright(c) 2009-2018, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 24/02/2018
    if nargin<3, isplot = 1; end
    if nargin<2, nsmooth = 5; end
    if isplot, myfigure; end
    ts = (od(end,end)-od(1,end))/length(od);
    if size(od,2)==4
        nv = normv(od(:,1:3));
        if isplot, plot(od(:,end), od(:,1:3)/ts); end
    else
        nv = od(:,1:end-1);
    end
    vel = nv/ts; sv = smooth(vel(:,1),nsmooth);
    if isplot
        hold on, plot(od(:,end), vel, '-', od(:,end), sv, ':b');
        xygo('OD Velocity / m/s');
        sumod = sum(nv);
        title(sprintf('Distance = %.4f (m) ', sumod));
    end
