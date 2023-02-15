function [dist, od] = distance(pos, timax)
% Get traveling distance from position(lat,lon,hgt) curve.
%
% Prototype: [dist, od] = distance(pos, timax)
% Inputs: pos - [lat, lon, hgt, t], must be sampled at same frequency.
%         timax - the max calculated time interval
% Outputs: dist - distances with different time interval
%          od - OD simulator output
%
% See also  pos2dxyz.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/12/2020
    if nargin<2, timax=1; end
    if size(pos,2)>7, pos = pos(:,[7:9,end]); end
    if size(pos,2)==3, pos(:,4) = (1:size(pos,1))'; end
    tmin = diff(pos(1:2,end));
    myfigure;
    subplot(2,2,[2,4]), dxyz=pos2dxyz(pos); plot(dxyz(:,1), dxyz(:,2)); xygo('E / m', 'N / m');
    hold on, plot(0,0,'or');
    subplot(223), plot(dxyz(:,end), dxyz(:,3)); xygo('hgt');
    subplot(221);
    for k=1:fix(log2(length(pos)))
        [RMh, clRNh] = RMRN(pos);
        dpos = [zeros(1,3);diff(pos(:,1:3))];
        dxyz = [dpos(:,2).*clRNh, dpos(:,1).*RMh, dpos(:,3)];
        if k==1, od = [normv(dxyz),pos(:,end)]; end
        distk = cumsum(normv(dxyz));
        plot(pos(:,end), distk); hold on;
        dist(k,1) = distk(end);
        pos = pos(1:2:end,:);
        tmax = diff(pos(1:2,end));
        if tmax>timax, break; end
    end
    xygo('distance / m');
    title(sprintf('Distance max=%.3fm(%.3fs), min=%.3fm(%.3fs)',dist(1),tmin,dist(end),tmax));
    

