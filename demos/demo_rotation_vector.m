% Yaw-Pitch-Roll Euler angles demostration
% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 25/01/2024
glvs
rv = [1.5;1;1.5];  rv = rv/norm(rv)*60*glv.deg;
x = [1;0;0]; y = [0;1;0]; z = [0;0;1];
len = 10;  xk = zeros(len,3); yk = xk; zk = xk;
myfig;
for k=1:len;
    rvk = rv/len*(k-1);
    qbn = rv2q(rvk);
    xk(k,:) = qmulv(qbn, x)';  yk(k,:) = qmulv(qbn, y)';  zk(k,:) = qmulv(qbn, z)';
    hold off;  a=0.9;
    plot3(a*xk(1:k,1), a*xk(1:k,2), a*xk(1:k,3), 'r:'); hold on;
    plot3(a*yk(1:k,1), a*yk(1:k,2), a*yk(1:k,3), 'm:');
    plot3(a*zk(1:k,1), a*zk(1:k,2), a*zk(1:k,3), 'b:');
    quiver3(0,0,0, rv(1),rv(2),rv(3),  'k','filled','LineWidth',3); text(rv(1),rv(2),rv(3)*1.1,'\phi');
    quiver3(0,0,0, 1,0,0,  'r','filled','LineWidth',2); text(1,0,0,'i_i');
    quiver3(0,0,0, 0,1,0,  'm','filled','LineWidth',2); text(0,1,0,'j_i');
    quiver3(0,0,0, 0,0,1,  'b','filled','LineWidth',2); text(0,0,1,'k_i');
    quiver3(0,0,0, xk(k,1),xk(k,2),xk(k,3),  'r-.','filled','LineWidth',2); text(xk(k,1),xk(k,2),xk(k,3),'i_b');
    quiver3(0,0,0, yk(k,1),yk(k,2),yk(k,3),  'm-.','filled','LineWidth',2); text(yk(k,1),yk(k,2),yk(k,3),'j_b');
    quiver3(0,0,0, zk(k,1),zk(k,2),zk(k,3),  'b-.','filled','LineWidth',2); text(zk(k,1),zk(k,2),zk(k,3),'k_b');
    axis equal; xylim([-1,1],3); xylabel('X','Y','Z'); grid on; view(118,25);
    if k==1; pause(2); end
    pause(0.2);
end

