% Noncommutativity-error motion simulation.
% See also  demo_cone_motion, demo_scull_motion.
% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/07/2020
glvs
Wt = randn(3,5);        % poly angular rate coefficients
ts = 0.05;              % sampling interval
hfig = figure('OuterPosition',[50,100,1100,500]); ki = 1;
xk = [1,0,0]; yk = [0,1,0]; zk = [0,0,1];
rot = zeros(1,10);
while 1
    t = ki*ts;  if t>1; break; end
    phi = btzpicard(Wt, t);
    phi1 = polyvaln(polyintn(polyadd(Wt, 0.5*polycross(polyintn(Wt),Wt))), t);
    wm = polyvaln(polyintn(Wt), t);
    qnb = rv2q(phi); % quaternion true value
    qbn = qconj(qnb);
    x = qmulv(qbn,[1;0;0]);    y = qmulv(qbn,[0;1;0]);    z = qmulv(qbn,[0;0;1]);
    xk(ki+1,:) = x'; yk(ki+1,:) = y'; zk(ki+1,:) = z';   % add point
    rot(ki+1,:) = [phi; phi1; wm; t]';
    if ~ishandle(hfig),  break;  end
    clf(hfig);  hold off;
    subplot(121);
    plot3([0,phi(1)],[0,phi(2)],[0,phi(3)],'-.bo','LineWidth',2); hold on; grid on % rotation vector
    plot3([0,x(1)],[0,x(2)],[0,x(3)],'-o', ...
          [0,y(1)],[0,y(2)],[0,y(3)],'-o', ...
          [0,z(1)],[0,z(2)],[0,z(3)],'-o','LineWidth',3); % body frame
    plot3([0,1.5],[0,0],[0,0],'m', ...
          [0,0],[0,1.5],[0,0],'m', ...
          [0,0],[0,0],[0,1.5], 'm'); % coordinate frames
    axis([-1.5 1.5 -1.5 1.5 -1.5 1.5]); view([1 1.2 1.1]);
    title('Noncommutativity-error Motion Simulation (by Yan G M)'); 
    xlabel('X'); ylabel('Y'); zlabel('Z'); 
    plot3(xk(:,1),xk(:,2),xk(:,3),':', ...
          yk(:,1),yk(:,2),yk(:,3),':', ...
          zk(:,1),zk(:,2),zk(:,3),':'); % tracks
    hl = legend('\phi_{ib}', 'x^b', 'y^b', 'z^b');  set(hl, 'Fontsize', 12);
    subplot(122);
    plot(rot(:,end), rot(:,1:3:9)*180/pi);
    hold on; plot(rot(:,end), rot(:,2:3:9)*180/pi,'--'); plot(rot(:,end), rot(:,3:3:9)*180/pi,':');
    xygo('Rotation / ( \circ )'); xlim([0,1]);
    legend('\phi_{x,real}', '\phi_{x,calcu}', '\Delta\theta_x', ...
        '\phi_{y,real}', '\phi_{y,calcu}', '\Delta\theta_y', ...
        '\phi_{z,real}', '\phi_{z,calcu}', '\Delta\theta_z', ...
        'Location','NorthEastOutside'); 
    pause(.1);
    ki = ki+1;
end
