% Somigliana Eq. & the simplified equation testing.
% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/01/2024
glvs;
Re = glv.Re; f = glv.f;  Rp = (1-f)*Re;
ge = glv.g0; m = Re*glv.wie^2/ge; beta = 5/2*m-f-17/14*m*f*1+15/4*m^2*0; gp = (1+beta)*ge;
beta1 = (5*m*f-f^2)/8;
L = (0:90)'*glv.deg;
g1 = (Re*ge*cos(L).^2+Rp*gp*sin(L).^2)./sqrt(Re^2*cos(L).^2+Rp^2*sin(L).^2);  % Somigliana
g2 = ge*(1+beta*sin(L).^2-beta1*sin(2*L).^2); % simplified model
b = lscov([sin(L).^2, -sin(2*L).^2], g1/ge-1);
g3 = ge*(1+b(1)*sin(L).^2-b(2)*sin(2*L).^2); % fitted model
[[beta;beta1], b],
myfig, subplot(121), plot(0:90, g1); xygo('L / \circ', 'g / m/s^2');
subplot(122), plot(0:90, [g2-g1,g3-g1]/glv.ug); xygo('L / \circ', '\deltag / ug');
legend('simplified error', 'fitted error');
return;

h = 10000; gg = [];
for k=1:91
    L = (k-1)*glv.deg;    pos = [L;0;h];
    [~,g] = wnieg(pos);
    RL = glv.Re*(1-glv.f*sin(pos(1))^2);  hR = pos(3)/RL;
    gL = glv.g0*(1+beta*sin(L).^2-beta1*sin(2*L).^2);
%     g = gL*(1-2*hR);
    gLh = gL*(1-2*hR-5*hR^2);
    gg(k,:) = [g, gLh];
end
myfig, subplot(121), plot(0:90, gg); xygo('L / \circ', 'g / m/s^2');
subplot(122), plot(0:90, (gg(:,2)-gg(:,1))/glv.ug); xygo('L / \circ', '\deltag / ug');

