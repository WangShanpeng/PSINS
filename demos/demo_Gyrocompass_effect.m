% Gyrocompass effect demostration
% See also  demo_gyro_rotor_precession, demo_sagnac_effect.
% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 20/01/2024
    glvs
    wnie = wnieg([10*glv.deg;0;0]);    F = -askew(wnie);
    phi0 = [0; 0; 10]*glv.deg;  vn = zeros(3,1);
    t = 0;  ts = 100;
    len = 1000;  kk = 1;
    phivn = zeros(1000,7);
    hfig = myfigure;   AZ=85; EL=10;
    while 1
        if ~ishandle(hfig),  break;  end
        clf(hfig);
        subplot(2,2,[1,3]); hold off;
        t = t+ts;
        phi = expm(F*t)*phi0;
        qnb = qaddphi([1;0;0;0], phi);
        vn = vn + qmulv(qnb, [0;0;-glv.g0]);
        x = [1;0;0];  y = [0;1;0];  z = [0;0;1];
        plot3([0,x(1)],[0,x(2)],[0,x(3)],'-*', [0,y(1)],[0,y(2)],[0,y(3)],'-*', [0,z(1)],[0,z(2)],[0,z(3)],'-*', 'LineWidth',1);  hold on;  grid on;
        x = qmulv(qnb, x);    y = qmulv(qnb, y);    z = qmulv(qnb, z);
        plot3([0,x(1)],[0,x(2)],[0,x(3)],'-o', [0,y(1)],[0,y(2)],[0,y(3)],'-o', [0,z(1)],[0,z(2)],[0,z(3)],'-o', 'LineWidth',3);
        title(sprintf('%.fs (%.2fh)', t,t/3600));
        xlim([-1.5,1.5]); ylim([-1.5,1.5]); zlim([-0.5,2.0]);
        view(AZ,EL);  % [AZ,EL] = view;
        phivn(kk,:) = [phi; vn; t];
        subplot(2,2,2), plot(phivn(1:kk,end), phivn(1:kk,1:3)/glv.deg);  xygo('\phi / ( \circ )');
            plot(phivn(1:kk,end), phivn(1:kk,1)/glv.deg, 'LineWidth',3);
            text(phivn(kk,end),phivn(kk,1)/glv.deg,'\phi_E');
        subplot(2,2,4), plot(phivn(1:kk,end), phivn(1:kk,4:5)); xygo('dV');
            plot(0,0,phivn(1:kk,end), phivn(1:kk,5), 'LineWidth',3);
            text(phivn(kk,end),phivn(kk,5),'\deltaV_N');
        kk = kk+1;
        if kk*ts>3600*24, break; end
        pause(0.05);
    end
    

