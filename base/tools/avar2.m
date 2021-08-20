function [sigma, tau, Err] = avar2(y0, tau0, yaw)
% Calculate Allan variance for two-column gyro data, and their addition&substraction.
%
% Prototype: [sigma, tau, Err] = avar2(y0, tau0, yaw)
% Inputs: y0 - two-column(X/Y) gyro data (in deg/hur)
%         tau0 - sampling interval
%         yaw - IMU yaw angle
%
% See also  avar, avars.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/04/2021
    if nargin<3, yaw=-45*pi/180; end
    cy = y0(:,1)*cos(yaw); sy = y0(:,2)*sin(yaw);
    y0 = [y0(:,1:2), cy-sy, cy+sy];
    str = ['-+b'; '-xk'; '-*r'; '-om'];
    myfig;
    sigma = []; tau = []; Err = [];
    for k=1:size(y0,2);
        [sigma(:,k), tau(:,k), Err(:,k)] = avar(y0(:,k), tau0, 0, 0);
        loglog(tau(:,k), sigma(:,k), str(k,:)); hold on
    end
    grid on;
    xlabel('\itt \rm/ s'); ylabel('\it\sigma_A\rm( \tau ) \rm (\circ)/h');
    legend('Xb','Yb','East','North');

