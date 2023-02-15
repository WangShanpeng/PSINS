function res = dataplot(data, clm, typestr)
% Data subset plot.
%
% Prototype: dataplot(data, typestr)
% Inputs: data - data to plot, the last column is time tag
%         clm - column index
%         typestr - type string to show in ylabel
%
% See also  xpplot, miniplot, plotn, avpidx, subavp.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/09/2021
global glv
    res = data(:,[clm,end]);
    switch typestr
        case 'imu',  imuplot(res);
        case 'avp',  insplot(res,typestr, 'mu');
        case 'avped',  insplot(res,typestr, 'mu');
        case 'v',  insplot(res,typestr);
        case 'vp',  insplot(res,typestr);
        case 'yaw',  myfig; plot(res(:,end),res(:,1)/glv.deg); xygo('y');
        case 'gps',  res=no0(res,1); gpsplot(res);
        case 'lever',  myfig; plot(res(:,end),res(:,1:3)); xygo('L');
        case 'dT',  myfig; plot(res(:,end),res(:,1)); xygo('dT');
        case 'od',   odplot(res);
        case 'odv',    myfig; plot(res(:,end),res(:,1:3)); xygo('dV');
        case 'kappa',  odpplot(res);
        case 'dKGzz',  myfig; plot(res(:,end),res(:,1)/glv.ppm); xygo('dKGzz / ppm');
        case 'temp',  templot(res);
    end