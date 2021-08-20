% Coning motion for spinning projectile simulation.
% See also  demo_cone_motion.
% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/01/2020
clear all
glvs
afa = 10.0*glv.deg;     % half-apex angle
f = 1;  w = 1*pi*f;     % coning frequency
w0 = 10*pi*f;           % rolling frequency
ts = 0.01;              % sampling interval
hfig = figure; ki = 1;
while 1
    t = ki*ts;
    qnb = [cos(afa/2)*cos(w0*t/2); sin(afa/2)*cos((w-w0/2)*t); sin(afa/2)*sin((w-w0/2)*t); cos(afa/2)*sin(w0*t/2)]; % quaternion true value
    x = qmulv(qnb,[1;0;0]);    y = qmulv(qnb,[0;1;0]);    z = qmulv(qnb,[0;0;1]);
    if t<2*pi/f % record first cycle
        xk(ki,:) = x'; yk(ki,:) = y'; zk(ki,:) = z';
    end
    if ~ishandle(hfig),  break;  end
    clf(hfig);  hold off;
    plot3([0,cos(w*t)],[0,sin(w*t)],[0,0],'-.bo','LineWidth',2); hold on; grid on % rotation vector
    plot3([0,1.5],[0,0],[0,0],'m', ...
          [0,0],[0,1.5],[0,0],'m', ...
          [0,0],[0,0],[0,1.5], 'm'); % coordinate frames
    axis([-1.5 1.5 -1.5 1.5 -1.5 1.5]); view([1 1.2 1.1]);
    title('Spinning Projectile Attitude Simulation (by Yan G M)'); 
    xlabel('X'); ylabel('Y'); zlabel('Z'); 
    plot3([0,x(1)],[0,x(2)],[0,x(3)],'-o', ...
          [0,y(1)],[0,y(2)],[0,y(3)],'-o', ...
          [0,z(1)],[0,z(2)],[0,z(3)],'-o','LineWidth',3); % body frame
    plot3(xk(:,1),xk(:,2),xk(:,3), ...
          yk(:,1),yk(:,2),yk(:,3), ...
          zk(:,1),zk(:,2),zk(:,3)); % tracks
    hl = legend('\it\bf\phi'); set(hl, 'Fontsize', 16);
    pause(.01);
    ki = ki+1;
end
