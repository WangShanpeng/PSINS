function posdr = drfusion(posdr1, posdr2, isinv, ext)
% Dead Reckoning(DR) pos fusion by distance.
%
% Prototype: posdr = drfusion(posdr1, posdr2, isinv, ext)
% Inputs: posdr1, posdr2 - DR pos trajectory
%         isinv - posdr2 is distance reverse
%         ext - distance extension before start & after end
% Output: posdr - DR pos after fusion.
%
% See also  drfit, drcalibrate, drrectify, odscale.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/02/2023
    if nargin<4, ext=1; end
    if nargin<3, isinv=0; end
    if isinv==1, posdr2=flipud(posdr2); end
    [dxyz, ddxyz, dist1] = pos2dxyz(posdr1, posdr1(1,1:3)');
    idx=(dist1(:,1)<1e-6);  posdr1(idx,:)=[];  dist1=cumsum(dist1(~idx,1));
    [dxyz, ddxyz, dist2] = pos2dxyz(posdr2, posdr2(1,1:3)');
    idx=(dist2(:,1)<1e-6);  posdr2(idx,:)=[];  dist2=cumsum(dist2(~idx,1));
    dt = 0.01;   dist = (max(dist1(1),dist2(1)):dt:min(dist1(end),dist2(end)))';   dist(end+1)=min(dist1(end),dist2(end));
    pos1 = interp1(dist1, posdr1(:,1:4), dist);
    pos2 = interp1(dist2, posdr2(:,1:3), dist);
	if isinv==1, posdr2=flipud(posdr2); end
    s=0.5; if isinv==1; s = repmat((1:length(pos1))'/length(pos1),1,3); end
    posdr = [(1-s).*pos1(:,1:3)+s.*pos2, pos1(:,4)];
    posstart = posdr(1:2,:); posend = posdr(end-1:end,:);  posstart(:,end)=posstart(1,end); posend(:,end)=posend(end,end);
    for k=1:fix(ext/dt)
        posstart = [posstart(1,:)+(posstart(1,:)-posstart(2,:)); posstart];
        posend(end+1,:) = posend(end,:)+(posend(end,:)-posend(end-1,:));
    end
    h = posdr(:,3);
    posdr = [posstart(1:end-2,:); posdr; posend(3:end,:)];
    xyz1 = pos2dxyz(posdr1,posdr1(1,1:3)');  xyz2 = pos2dxyz(posdr2,posdr1(1,1:3)'); xyz = pos2dxyz(posdr,posdr1(1,1:3)');
    myfig,
    subplot(121), plot(xyz1(:,1), xyz1(:,2), xyz2(:,1), xyz2(:,2), xyz(:,1), xyz(:,2)); xygo('E / m', 'N / m'); legend('DRforward', 'DRbackward', 'DRfusion');
        hold on, plot(xyz1(1,1), xyz1(1,2), 'o', xyz1(end,1), xyz1(end,2), 'o');
    subplot(122), plot(dist, pos1(:,3), dist, pos2(:,3), dist, h); xygo('dist / m', 'h / m');
%     pos2dplot(posdr1, posdr2, posdr);  legend('Start', 'DR1', 'DR2', 'DRfusion');
%     myfig, plot(dist, pos1(:,3), dist, pos2(:,3), dist, h); xygo('dist / m', 'h / m');