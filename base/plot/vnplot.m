function vb = vnplot(av, thvel, thdyaw)
% DVL or OD plot.
%
% Prototype: dvlplot(dvl)
% Inputs: dvl - [vb_right, vb_front, vb_up, t] or [vb_front, t]
%         sf - scale factor
%          
% See also  odplot, dvlplot, vn2vbl.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 31/12/2020
global glv
    if nargin<3, thdyaw=5.0; end
    if nargin<2, thvel=1.0; end
    vb = vn2vbl(av(:,3), av(:,4:6));
    myfigure,
    subplot(221), plot(av(:,end), av(:,4:6)); xygo('vn / m/s');
    subplot(222), plot(av(:,end), vb); xygo('vb / m/s');
    wz = [0;diff(av(:,3))/diff(av(1:2,end))]/glv.deg; idxwz = abs(wz)<50;
    subplot(223), plot(av(:,end), vb(:,1)*10, av(idxwz,end), wz(idxwz)); xygo('vx / x0.1m/s, wz / \circ/s');
    vxwz = vb(:,1)./(wz*glv.deg);  idxvw = abs(wz)>3;
    hold on, plot(av(idxvw,end), vxwz(idxvw), 'r');
    dy = [thdyaw; diff(av(:,3))/diff(av(1:2,end))/(pi/180)];
    idx = abs(vb(:,2))>thvel & abs(dy)<thdyaw;
    ang = [atan2(vb(idx,3),vb(idx,2))-av(idx,1), -atan2(vb(idx,1),vb(idx,2))]/glv.deg;
	subplot(224), plot(av(idx,end), [smooth(ang(:,1),100),smooth(ang(:,2),100)]);
    xygo('\delta\theta,\delta\Psi / (\circ)');    ylim([-thdyaw,thdyaw]);

