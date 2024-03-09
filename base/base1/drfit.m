function posdr = drfit(posdr, pos0, pos1, isfig)
% Dead Reckoning(DR) parameter calibration for 'inst'&'kod'.
%
% Prototype: posdr = drfit(posdr, pos0, pos1)
% Inputs: posdr - DR pos trajectory
%         pos0 - pos Marker0
%         pos1 - pos Marker1
%         isfi - figure flag
% Output: posdr - DR pos after modification.
%
% See also  drcalibrate, drrectify, odscale.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/02/2023
    if nargin<4, isfig=0; end
    for k=1:3,  posdr(:,k) = posdr(:,k)-posdr(1,k)+pos0(k);  end
    pos0 = posdr(1,1:3)';
    [inst, kod] = drcalibrate(pos0, pos1(1:3), posdr(end,1:3)');
    [dxyz,dist] = pos2dxyz(posdr); dxyz=dxyz*kod; dist=dist*kod;  dxyz1=pos2dxyz([pos0'; pos1(1:3)']);
    dxyz(:,1:3) = dxyz(:,1:3)*a2mat([0;0;inst(3)]);
    dist = cumsum(normv(dist(:,1:2))); dh = dxyz(end,3)-dxyz1(end,3) - (dxyz(1,3)-dxyz1(1,3));  % height fusion
% 	dxyz(:,3) = dxyz(1,3) + dxyz(:,3) - (dxyz(:,3)-dxyz(1,3))*dh/(dxyz(end,3)-dxyz(1,3)).*dist/dist(end);
	dxyz(:,3) = dxyz(1,3) + dxyz(:,3) - dh*dist/dist(end);
    h0=posdr(:,3); posdr = dxyz2pos(dxyz, pos0);
    if isfig==1
        myfig,
        subplot(121), plot(dxyz(:,1), dxyz(:,2), '-', dxyz1(:,1), dxyz1(:,2), 'o');  xygo('x / m', 'y / m');
        subplot(122), plot(dist, posdr(:,3), dist, h0); xygo('dist / m', 'h / m');  plot(dist(1),pos0(3), 'o', dist(end), pos1(3), 'o');
    end
