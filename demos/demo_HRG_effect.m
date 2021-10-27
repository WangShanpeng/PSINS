% Hemispherical Resonator Gyro demostration
% See also  demo_gyro_rotor_precession, demo_sagnac_effect.
% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 18/07/2021
    glvs
    w = 1;   % 0 for static, 1 for clockwise, -1 for counter clockwise
    t = 0;  dr = 0; ddr = 0.01;  afa = (0:400)'*glv.deg;  K = 0.277;
    hfig = figure;
    while 1
        if ~ishandle(hfig),  break;  end
        clf(hfig);  hold off;
        t = t+0.1;   afa0 = w*t*glv.dps;
        safa = sin(afa+afa0); cafa = cos(afa+afa0);
        safa1 = sin(afa+afa0*(1-K)); cafa1 = cos(afa+afa0*(1-K));
        plot(1.1*safa, 1.1*cafa, ':m', safa, cafa, 'b'); % HR
        xlim([-1.5,1.5]);  ylim([-1.5,1.5]); axis equal; hold on; grid on;
        plot(0,0,'or', [0,safa(1)],[0,cafa(1)],':b', [0,1.1*safa1(1)],[0,1.1*cafa1(1)],':m');
        plot(1.1*safa1+dr*sin(3*afa+afa0/3), 1.1*cafa1+dr*cos(3*afa+afa0/3), 'm'); % HR
        for k=1:8
            k1 = k*45-10; k2 = k*45+10;
            plot(safa(k1:k2), cafa(k1:k2), 'linewidth',4);  % base
        end
        dr = dr+ddr;
        if dr>0.05, ddr=-0.01; elseif dr<-0.05, ddr=0.01; end
        pause(0.01);
    end
