function pos = extrapos(pos, dis)
% Extract pos by specific distance interval.
%
% Prototype: pos = extrapos(pos, dis)
% Inputs: pos - input pos [lat, lon, hgt, t]
%         dis - distance interval
% Output: pos - out pos
%
% See also  pos2dxyz.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 13/08/2022
    if nargin<2, dis = 100; end;
    [dxyz,ddxyz] = pos2dxyz(pos(:,end-3:end), pos(1,end-3:end-1)');
    cdis = cumsum(normv(ddxyz(:,1:3)));  % cumulative distance
    t = dxyz(:,end); tmp = cdis(1);
    kk = 2;
    for k=2:length(cdis)
        if cdis(k)-tmp>dis, t(kk)=dxyz(k,end); kk=kk+1; tmp=cdis(k); end
    end
    t(kk) = dxyz(end,end);
    t(kk+1:end) = [];
    [~,i1,i2] = intersect(pos(:,end), t);
    pos = pos(i1,:);

