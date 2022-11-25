% Rigid body angular motion simulation.
% See also  demo_gyro_rotor_precession.
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 10/10/2022
function demo_rigid_body_angular_motion
    ts = 0.01;  T = 10;  len = fix(T/ts);
    R = [0.5; 1.5; 4]; M = [0.001; 0; 0]*10; w = [0; 0; 00];  % J=1./R^2 moment of inertia; M = moment of force; % w = initial angular rate
    R = [3.5; 3.5; 1]; M = zeros(3,1); w = [0; 1; 3]; % nutation
%     R = [0.5; 1.5; 4]; M = zeros(3,1); w = [0.001; 5; 0]; % Dzhanibekov effect
    Cnb = eye(3); % initial attitude
    [x,y,z] = ellipsoid(0,0,0, R(1),R(2),R(3), 16);
    n = length(x); n2 = n*n; m = ceil(n/2);
    xyz0 = [reshape(x,n2,1),reshape(y,n2,1),reshape(z,n2,1)]; c = ones(n,n);
    hfig = figure;
    for k=1:len
        if k>50, M=0; end
        w1 = rk4(w, 1./R.^2, 0, M, ts);  % J=1./R^2
        Cnb = mupdt(Cnb, (w+w1)*ts/2); w = w1;
        xyz = xyz0*Cnb';
        x = reshape(xyz(:,1),n,n); y = reshape(xyz(:,2),n,n); z = reshape(xyz(:,3),n,n);
        if ~ishandle(hfig),  break;  end;  hold off; 
        surf(x(1:m,1:m),y(1:m,1:m),z(1:m,1:m),c(1:m,c:m)*0.1); hold on
        surf(x(1:m,m:n),y(1:m,m:n),z(1:m,m:n),c(1:m,m:n)*0.4);
        surf(x(m:n,1:m),y(m:n,1:m),z(m:n,1:m),c(m:n,1:m)*0.6);
        surf(x(m:n,m:n),y(m:n,m:n),z(m:n,m:n),c(m:n,m:n)*0.9);
        plot3([-5,5],[0,0],[0,0],'-.b'); plot3([0,0],[-5,5],[0,0],'-.g'); plot3([0,0],[0,0],[-5,5],'-.r');
        xlabel('X'); ylabel('Y'); zlabel('Z');
        title(sprintf('t=%.2fs; w=%.2f, %.2f, %.2f, %.2f(rad/s)  by Yan', k*ts,w(1),w(2),w(3),norm(w)));
        pause(ts/2);
    end

function w1 = rk4(w, J, Omega, M, ts)
	k1 = rk(w, J, Omega, M);
	k2 = rk(w+ts/2*k1, J, Omega, M);
	k3 = rk(w+ts/2*k2, J, Omega, M);
	k4 = rk(w+ts*k3, J, Omega, M);
	w1 = w+ts/6.*(k1+2*(k2+k3)+k4);

function ki = rk(w, J, Omega, M)
    Jww = [ (J(3)-J(2))*w(2)*w(3);
            (J(1)-J(3))*w(1)*w(3)+J(1)*Omega*w(3);
            (J(2)-J(1))*w(2)*w(1)-J(1)*Omega*w(2) ];
    ki = (M-Jww)./J;
