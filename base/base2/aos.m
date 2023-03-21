function aos(av, aosk, vT, dyawI)
% Angle Of Slide calculate & display
%
% Prototype: aos(av, aosk, vT, dyawI)
% Inputs: av - att & vel  = [att; vel; t] array
%         aosk - ratio
%         vT - vel threshold
%         dyawI - yaw install error
%
% See also  att2c.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 11/03/2023
global glv;
    if nargin<4, dyawI = 0; end
    if nargin<3, vT = 1; end
    if nargin<2, aosk = 1/50; end
    a = att2c(av(:,[1:3,end]));
    wz = diff(a([1,1:end],3))/diff(a(1:2,end));
    att1 = vn2att(av(:,[4:6,end]));
    vel = normv(av(:,4:5));
    dyaw = diffyaw(av(:,3), att1(:,3));
    idx = vel>vT;
    myfig,
    subplot(311), plot(av(:,end), [av(:,1:2)*10, av(:,3), att1(:,3)]/glv.deg); grid on; ylabel('Att / \circ');  legend('\theta*10', '\gamma*10', '\psi','\psi_{track}'); x=get(gca,'xlim');
%     subplot(311), plot(av(idx,end), [av(idx,1:2)*10, av(idx,3), att1(idx,3)]/glv.deg); grid on; ylabel('Att / \circ');  legend('\theta*10', '\gamma*10', '\psi','\psi_{track}');
    subplot(312), plot(av(idx,end), [wz(idx)/glv.dps,vel(idx)]); grid on; legend('\omega_z / \circ/s', 'vel / m/s'); xlim(x);
    subplot(313), plot(av(idx,end), [dyaw(idx)-dyawI,-aosk*vel(idx).*wz(idx)]/glv.deg); ylim([-10,10]);  xygo('AOS');  legend('True','Simu'); xlim(x);


