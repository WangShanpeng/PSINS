% MEMS-gyro Coriolis effect demostration
% See also  demo_gyro_rotor_precession, demo_sagnac_effect.
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 18/11/2022
    glvs
    t = 0;
    xy01 = [1 -1 -1 1 1; 1 1 -1 -1 1];  xy02 = [xy01(1,:)*0.1; xy01(2,:)*0.9];  % frame
    hfig = figure;
    while 1
        if ~ishandle(hfig),  break;  end
        clf(hfig);  hold off;
        t = t+0.1;   afa0 = 3*t*glv.dps;  ca=cos(afa0);sa=sin(afa0);  C = [ca, -sa; sa, ca];
        xy1 = C * xy01;  xy2 = C * xy02;
        a = 0.8; y = a*sin(2*pi*0.1*t);
        pt0 = [0; y];  pt1 = [pt0(1)+0.1; y]; pt2 = [-pt1(1); y];
        pt0 = C * pt0; pt1 = C * pt1; pt2 = C * pt2;
        hold off; plot(xy1(1,:), xy1(2,:), xy2(1,:),xy2(2,:), 'linewidth', 3);
        hold on, plot(pt0(1), pt0(2), 'o', 'linewidth', 6);
        if y>0, clr1=y/a; else clr1=0; end
        if y<0, clr2=-y/a; else clr2=0; end
        plot(pt1(1), pt1(2), '*', 'linewidth', 5,  'color', [clr1;clr2;clr2]);
        plot(pt2(1), pt2(2), '*', 'linewidth', 5,  'color', [clr2;clr1;clr1]);
        xlim([-2,2]); ylim([-2,2]); title('MEMS-gyro Coriolis effect'); axis equal;
        pause(0.05);
    end
