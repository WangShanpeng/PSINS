function posdr = drfit(posdr, pos0, pos1, isfig)
% Dead Reckoning(DR) parameter calibration for 'inst'&'kod'.
%
% Prototype: posdr = drfit(posdr, pos0, pos1)
% Inputs: posdr - DR pos trajectory
%         pos0 - pos Marker0. if exist pos0(4), it is the corresponding index for posdr(idx0,1:3)
%         pos1 - pos Marker1. if exist pos1(4), it is the corresponding index for posdr(idx1,1:3) 
%         isfig - figure flag
% Output: posdr - DR pos after modification.
%
% See also  drcalibrate, drrectify, odscale, drfusion.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/02/2023, 17/08/2024
    if nargin<4, isfig=0; end
    if length(pos0)>3, idx0=pos0(4);  else, idx0=1; end
    for k=1:3,  posdr(:,k) = posdr(:,k)-posdr(idx0,k)+pos0(k);  end
    pos0 = posdr(idx0,1:3)';
    if length(pos1)>3, idx1=pos1(4);  else, idx1=length(posdr); end
    [inst, kod] = drcalibrate(pos0, pos1(1:3), posdr(idx1,1:3)');
    [dxyz0,dist] = pos2dxyz(posdr); dxyz=dxyz0*kod; dist=dist*kod;  dxyz1=pos2dxyz([pos0'; pos1(1:3)']);
    dxyz(:,1:3) = dxyz(:,1:3)*a2mat([0;0;inst(3)]);
    dist = cumsum(normv(dist(:,1:2))); dh = dxyz(idx1,3)-dxyz1(end,3) - (dxyz(idx0,3)-dxyz1(1,3));  % height fusion
% 	dxyz(:,3) = dxyz(1,3) + dxyz(:,3) - (dxyz(:,3)-dxyz(1,3))*dh/(dxyz(end,3)-dxyz(1,3)).*dist/dist(end);
	dxyz(:,3) = dxyz(idx0,3) + dxyz(:,3) - dh*dist/(dist(idx1)-dist(idx0));
    h0=posdr(:,3); posdr = dxyz2pos(dxyz, pos0);
    if isfig==1
        myfig,
        subplot(121), plot(dxyz0(:,1), dxyz0(:,2), '--', dxyz(:,1), dxyz(:,2), '-', dxyz1(:,1), dxyz1(:,2), 'o');  xygo('x / m', 'y / m');
        subplot(122), plot(dist, h0, dist, posdr(:,3)); xygo('dist / m', 'h / m');  plot(dist(idx0),pos0(3), 'o', dist(idx1), pos1(3), 'o');
    end
