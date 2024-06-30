% Yaw-Pitch-Roll Euler angles demostration
% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 25/01/2024
glvs
r = [.5; 0; .7];  u = [0;0;1]*60*glv.deg;
len = 10;  rp = zeros(len,3);
% myfig;
for k=1:len;
    uk = u/len*(k-1);
    qbn = rv2q(uk);
    rp(k,:) = qmulv(qbn, r)';
    hold off;  a=0.9;
    plot3(a*rp(1:k,1), a*rp(1:k,2), a*rp(1:k,3), 'r:'); hold on;
    quiver3(0,0,0, rp(k,1),rp(k,2),rp(k,3),  'b-.','filled','LineWidth',2); text(rp(k,1)/2,rp(k,2)/2,rp(k,3)/2+0.05,'r\prime'); text(rp(k,1),rp(k,2),rp(k,3),'A\prime');
    quiver3(0,0,0, u(1),u(2),u(3),  'k','filled','LineWidth',3); text(u(1),u(2),u(3),'u');
    quiver3(0,0,0, r(1),r(2),r(3),  'r','filled','LineWidth',2); text(r(1)/2,r(2)/2,r(3)/2+0.05,'r'); text(r(1),r(2),r(3),'A');
    axis equal; xylim([-1,1],[-1,1],[-.2,1]); xylabel('X','Y','Z'); grid on; view(-140,20);
    pause(0.2);
end
plot3([0,a*rp(1,1)],[0,0],a*[rp(1,3),rp(1,3)], 'm'); % text(0,0,0.3, ' (r.u)u');
afa = (0:1*glv.deg:2*pi)'; b = norm(r(1:2));
rp1 = [b*cos(afa), b*sin(afa), repmat(r(3),length(afa),1)];
plot3(a*rp1(:,1), a*rp1(:,2), a*rp1(:,3), 'm:');
plot3([0,0],[0,a*rp(1,1)],a*[rp(1,3),rp(1,3)], 'm');
plot3([0,a*rp(k,1)],[0,a*rp(k,2)],a*[rp(1,3),rp(1,3)], 'm');

r=randn(3,1); u=randn(3,1); u=cross(r,u); u=u/norm(u);
phi=randn(1);
r+(1-cos(phi))*askew(u)^2*r - r*cos(phi)
