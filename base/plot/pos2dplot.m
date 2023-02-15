function xyz = pos2dplot(pos0, varargin)
% Multi-pos 2D trajectory plot.
%
% Prototype: xyz = pos2dplot(pos0, varargin)
% Inputs: pos0 - [lat, lon, hgt, t]
%         varargin - other pos parameter
% Onput: xyz - [x/East, y/North, hgt, t]
%          
% See also  pos3vplot, inserrplot, kfplot, gpsplot, imuplot, pos2dxyz, pos2bd.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 24/05/2015
    if nargin<2, varargin{1}=0; end
    if length(varargin{end})==1, aequal=1; varargin(end)=[]; else, aequal=0; end  % for axis equal
    if size(pos0,2)>7, pos0=pos0(:,[7:9,end]); end   % avp->pos
    for k=1:length(varargin); 
        if size(varargin{k},2)>7, varargin{k}=varargin{k}(:,[7:9,end]); end
    end
    if size(pos0,2)>4, pos0=pos0(:,[4:6,end]); end   % vp->pos
    for k=1:length(varargin); 
        if size(varargin{k},2)>4, varargin{k}=varargin{k}(:,[4:6,end]); end
    end
    lat0 = pos0(1,1); lon0 = pos0(1,2);
    eth = earth(pos0(1,1:3)');
    myfig;
    plot(0, 0, 'bp'); hold on;
    xyz = [(pos0(:,2)-lon0)*eth.clRNh, (pos0(:,1)-lat0)*eth.RMh, pos0(:,3:4)];
    plot(xyz(:,1), xyz(:,2))
    if nargin>1 xy1 = xyz; xyz = []; xyz{1} = xy1; end
    if nargin>1  % pos2dplot(pos0, pos1, pos2, ...)
        clr = 'rmycgb';
        for k=1:length(varargin)
            pos0 = varargin{k};
            xyz{k+1} = [(pos0(:,2)-lon0)*eth.clRNh, (pos0(:,1)-lat0)*eth.RMh, pos0(:,3:4)];
            plot(xyz{k+1}(:,1), xyz{k+1}(:,2), clr(k));
        end
    end
    xygo('est', 'nth');
	legend(sprintf('%.2f, %.2f / DMS', r2dms(lat0),r2dms(lon0)));
    if aequal==1, axis equal; end

