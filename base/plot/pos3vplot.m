function pos3vplot(pos0, varargin)
% Three-view drawing plot
%
% Prototype: pos3vplot(pos0, varargin)
% Input2: pos0,varargin - [lat,lon,hgt,t] position array
%
% Example:
%    t=(1:100)'; pos=[ 0.5+0.001*sin(0.1*t+0.3), 1.0+0.001*cos(0.2*t), t, t]; pos3vplot(pos);
%
% See also  pos2dplot, pos2dxyz.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/05/2020, 03/04/2021
    myfig,
    xyz = pos2dxyz(pos0);
    subplot(221), plot(xyz(:,1), xyz(:,3)); xygo('East / m', 'Up / m'); title('South->North View');
    subplot(222), plot(xyz(:,2), xyz(:,3)); xygo('North / m', 'Up / m'); title('West->East View');  set(gca,'XDir','reverse')
    subplot(223), plot(xyz(:,1), xyz(:,2)); xygo('East / m', 'North / m'); title('Up->Down View');
    if nargin>1
        clr = 'rmycgb';
        for k=1:length(varargin)
            pos = varargin{k}; for kk=1:3, pos(:,kk)=pos(:,kk)+(pos0(1,kk)-pos(1,kk)); end
            xyz = pos2dxyz(pos);
            subplot(221), plot(xyz(:,1), xyz(:,3), clr(k));
            subplot(222), plot(xyz(:,2), xyz(:,3), clr(k));
            subplot(223), plot(xyz(:,1), xyz(:,2), clr(k));
        end
    end
