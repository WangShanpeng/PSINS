function [pos, idx] = posdelol(pos, latol, lonol, hgtol)
% Delete pos(lat,lon,hgt) outliers, default applied for pos within China(73~135E¡¢3~54N).
%
% Prototype:  pos = posbias(pos,b)
% Inputs: pos - input pos data
%         latol - latitude outlier range
%         lonol - longitude outlier range
%         hgtol - height outlier range
% Outputs: pos - output pos data without outlier
%          idx - outlier index
%
% See also  delbias, posbias.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 14/08/2024
global glv
    if nargin<4, hgtol=[]; end
    if nargin<3, lonol=[]; end
    if nargin<2, latol=[]; end
    if length(latol)==1
        if isempty(hgtol), hgtol=1000; end
        if isempty(lonol), lonol=latol; end
        abslat=abs(pos(:,end-3)); idx1 = abslat>0 & abslat<pi/2;
        abslon=abs(pos(:,end-2)); idx2 = abslon>0 & abslon<=pi;
        abshgt=abs(pos(:,end-1)); idx3 = abshgt<=100000;
        idx = idx1&idx2&idx3;
        posm = [median(pos(idx,end-3)), median(pos(idx,end-2)), median(pos(idx,end-1))];
        abslat=abs(pos(:,end-3)-posm(1)); idx1 = abslat<latol;
        abslon=abs(pos(:,end-2)-posm(2)); idx2 = abslon<lonol;
        abshgt=abs(pos(:,end-1)-posm(3)); idx3 = abshgt<hgtol;
        idx = idx1&idx2&idx3;
        pos = pos(idx,:);
        idx = find(idx==0);
        return;
    end
    if isempty(hgtol), hgtol=[-1000,10000]; end
    if isempty(lonol), lonol=[73,135]*glv.deg; end
    if isempty(latol), latol=[3,54]*glv.deg; end
    idx = pos(:,end-3)<latol(1) | pos(:,end-3)>latol(2) | ...
          pos(:,end-2)<lonol(1) | pos(:,end-2)>lonol(2) | ...
          pos(:,end-1)<hgtol(1) | pos(:,end-1)>hgtol(2) ;
    pos(idx,:) = [];
    idx = find(idx);