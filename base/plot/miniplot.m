function miniplot(data, str)
% A mini plot function
%
% Prototype: miniplot(data, str)
% Inputs: data - data to plot, 
%         str - flag string
%
% See also  dataplot, plotn, insplot, labeldef, mlplot, msplot.

% Copyright(c) 2009-2017, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/07/2017
global glv
    sz = size(data,2);  sz1=sz-1;
    myfig;
    switch str
        case 'w',
            if sz>4, sz=4; data=data(:,[1:3,end]); end
            sz1=3; data(:,1:sz1)=data(:,1:sz1)/diff(data(1:2,end))/glv.dps;
        case 'wx',
            if sz>4, sz=2; data=data(:,[1,end]); end
            sz1=1; data(:,1)=data(:,1)/diff(data(1:2,end))/glv.dps;
        case 'wy',
            if sz>4, sz=2; data=data(:,[2,end]); end
            sz1=1; data(:,1)=data(:,1)/diff(data(1:2,end))/glv.dps;
        case 'wz',
            if sz>4, sz=2; data=data(:,[3,end]); end
            sz1=1; data(:,1)=data(:,1)/diff(data(1:2,end))/glv.dps;
        case 'f',
            if sz>6, sz=4; data=data(:,[4:6,end]); end
            sz1=3; data(:,1:sz1)=data(:,1:sz1)/diff(data(1:2,end))/glv.g0;
        case 'fx',
            if sz>6, sz=2; data=data(:,[4,end]); end
            sz1=1; data(:,1)=data(:,1)/diff(data(1:2,end))/glv.g0;
        case 'fy',
            if sz>6, sz=2; data=data(:,[5,end]); end
            sz1=1; data(:,1)=data(:,1)/diff(data(1:2,end))/glv.g0;
        case 'fz',
            if sz>6, sz=2; data=data(:,[6,end]); end
            sz1=1; data(:,1)=data(:,1)/diff(data(1:2,end))/glv.g0;
        case 'p',
            if sz>2, sz=2; data=data(:,[1,end]); end
            sz1=1; data(:,1:sz1)=data(:,1:sz1)/glv.deg;
        case 'r',
            if sz>2, sz=2; data=data(:,[2,end]); end
            sz1=1; data(:,1:sz1)=data(:,1:sz1)/glv.deg;
        case 'pr',
            if sz>3, sz=3; data=data(:,[1:2,end]); end
            sz1=2; data(:,1:sz1)=data(:,1:sz1)/glv.deg;
        case {'y','yaw'},
            if sz>3, sz=2; data=data(:,[3,end]); end
            sz1=1; data(:,1:sz1)=data(:,1:sz1)/glv.deg;
        case 'att',
            if sz>4, sz=4; data=data(:,[1:3,end]); end
            sz1=3; data(:,1:sz1)=data(:,1:sz1)/glv.deg;
        case 'VEN',
            if sz>3, sz=3; data=data(:,[4:5,end]); end
            sz1=2;
        case 'VU',
            if sz>2, sz=2; data=data(:,[6,end]); end
            sz1=1;
        case 'vn',
            if sz>4, sz=4; data=data(:,[4:6,end]); end
            sz1=3;
        case 'pos',
            if sz>4, sz=4; data=data(:,[7:9,end]); end
            data = pos2dxyz(data); data(:,1:2) = data(:,[2,1]);
            sz1=3;
        case 'hgt',
            if sz>2, sz=2; data = data(:,end-1:end); end
            sz1=1;
        case 'phi',
            if sz>4, sz=4; data=data(:,[1:3,end]); end
            sz1=3; data(:,1:sz1)=data(:,1:sz1)/glv.min;
        case 'dV',
            if sz>4, sz=4; data=data(:,[4:6,end]); end
            sz1=3;
        case 'dP',
            if sz>4, sz=4; data=data(:,[7:9,end]); end
            sz1=3; data(:,1:sz1-1)=data(:,1:sz1-1)*glv.Re;
        case 'eb',
            if sz>4, sz=4; data=data(:,[10:12,end]); end
            sz1=3; data(:,1:sz1)=data(:,1:sz1)/glv.dph;
        case 'db',
            if sz>4, sz=4; data=data(:,[13:15,end]); end
            sz1=3; data(:,1:sz1)=data(:,1:sz1)/glv.ug;
        case 'L',
            if sz>4, sz=4; data=data(:,[16:18,end]); end
            sz1=3;
        case 'dT',
            if sz>2, sz=2; data=data(:,[19,end]); end
            sz1=1;
    end
    if sz==sz1, plot(data(:,1:sz1)); 
    else, plot(data(:,end), data(:,1:sz-1)); end
    xygo(str);
